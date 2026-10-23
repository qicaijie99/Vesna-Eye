// Target tracker: orchestrates detector -> PnP -> Kalman filters + state machine.
//
// Per frame:
//   1. predict both Kalman filters to the current timestamp,
//   2. choose the detector ROI:
//        SEARCH / ACQUIRE / TRACK / PREDICT -> predicted LOS + apparent target
//          size with padding, clamped to the image (empty rect means "full
//          frame" for the detector; SEARCH has no prediction yet and therefore
//          always searches the full frame on every frame -- the full-frame
//          detection costs ~2 ms at 720p, well inside the frame budget),
//        LOST -> full frame, but throttled by full_search_interval frames,
//   3. run the detector, association-gate the detection against the predicted
//      image position, run PnP on accepted detections,
//   4. feed angles/range to the filters (the angle filter has its own
//      chi-square innovation gate), update the state machine,
//   5. emit the (latency-compensated) angles. While PREDICTing the filtered
//      angles are extrapolated and `quality` decays with every miss.
//
// State machine:
//   SEARCH  --(hit)--> ACQUIRE --(acquire_hits consecutive hits)--> TRACK
//   TRACK   --(miss)--> PREDICT --(hit)--> TRACK
//                       PREDICT --(max_miss_frames misses)--> LOST --> SEARCH
// (full_search_interval only throttles the LOST re-search, never SEARCH.)
//
// Conventions (see types.hpp): yaw = atan2(t.x, t.z) positive right,
// pitch = -atan2(t.y, t.z) positive up, distance = ||t||.

#include "glowtrack/tracker.hpp"

#include "glowtrack/detector.hpp"
#include "glowtrack/kalman_filter.hpp"
#include "glowtrack/pose_estimator.hpp"
#include "glowtrack/util.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace glowtrack {
namespace {

constexpr double kMaxDtSeconds = 0.5;  // clamp for frame-gap outliers
constexpr double kMinAssociationPx = 24.0;
constexpr double kAssociationRadiusScale = 2.5;
constexpr double kMinApparentRadiusPx = 6.0;
constexpr double kMaxSigmaScale = 6.0;
constexpr double kMinRoiIntersectionPx = 24.0;

// Projects a line-of-sight (yaw, pitch) direction into the image. The point is
// placed on the ray z = 1 and pushed through cv::projectPoints so the
// configured lens distortion is honoured. Returns false for degenerate angles.
bool projectLos(const CameraIntrinsics& camera, double yaw, double pitch,
                cv::Point2f* pixel) {
  if (pixel == nullptr || !std::isfinite(yaw) || !std::isfinite(pitch) ||
      std::fabs(yaw) > 1.45 || std::fabs(pitch) > 1.45) {
    return false;
  }
  std::vector<cv::Point3d> object_points;
  object_points.emplace_back(std::tan(yaw), -std::tan(pitch), 1.0);
  std::vector<cv::Point2d> image_points;
  try {
    cv::projectPoints(object_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                      camera.cameraMatrix(), camera.distortion(),
                      image_points);
  } catch (const cv::Exception&) {
    return false;
  }
  if (image_points.size() != object_points.size()) {
    return false;
  }
  *pixel = cv::Point2f(static_cast<float>(image_points[0].x),
                       static_cast<float>(image_points[0].y));
  return std::isfinite(pixel->x) && std::isfinite(pixel->y);
}

double discRadiusMeters(const RuntimeConfig& config) {
  return std::max(1e-4, 0.5 * config.target.disc_diameter_mm / 1000.0);
}

}  // namespace

struct TargetTracker::Impl {
  explicit Impl(const RuntimeConfig& cfg)
      : config(cfg),
        detector(cfg.detector, cfg.target),
        pose_estimator(cfg.camera, cfg.target, cfg.pose),
        angle_filter(cfg.kalman),
        range_filter(cfg.kalman) {}

  RuntimeConfig config;
  TargetDetector detector;
  PoseEstimator pose_estimator;
  AngleKalmanFilter angle_filter;
  RangeKalmanFilter range_filter;

  TrackState state = TrackState::Searching;
  Detection last_detection;
  PoseEstimate last_pose;
  cv::Rect search_roi;
  double last_processing_ms = 0.0;

  bool has_last_timestamp = false;
  double last_timestamp_s = 0.0;
  int consecutive_hits = 0;
  int consecutive_misses = 0;
  int frames_since_full_search = 0;
  double quality = 0.0;
  double apparent_radius_px = 0.0;

  // --- helpers -------------------------------------------------------------

  double decayPerFrame() const {
    const double decay = config.kalman.quality_decay_per_frame;
    return (std::isfinite(decay) && decay > 0.0 && decay <= 1.0) ? decay : 0.9;
  }

  int acquireHits() const {
    return std::max(1, config.tracker.acquire_hits);
  }

  int maxMissFrames() const {
    return std::max(1, config.tracker.max_miss_frames);
  }

  // Best estimate of the target's apparent disc radius in pixels. Uses the
  // last accepted detection when available, falls back to the range filter and
  // finally to the configured ROI minimum, and grows while coasting.
  double predictedRadiusPx() const {
    double radius = apparent_radius_px;
    if (!(radius > 0.0) && range_filter.initialized()) {
      radius = config.camera.fx * discRadiusMeters(config) /
               std::max(1e-3, range_filter.range());
    }
    if (!(radius > 0.0)) {
      radius = std::max(8.0, 0.5 * config.tracker.roi_min_size_px);
    }
    if (state == TrackState::Predicting) {
      const double growth = config.tracker.prediction_growth_per_frame;
      const double safe_growth =
          (std::isfinite(growth) && growth >= 1.0) ? growth : 1.15;
      radius *= std::pow(safe_growth, std::max(1, consecutive_misses));
    }
    return std::max(kMinApparentRadiusPx, radius);
  }

  cv::Point2f predictedPixel() const {
    cv::Point2f pixel(-1.f, -1.f);
    if (!angle_filter.initialized()) {
      return pixel;
    }
    if (!projectLos(config.camera, angle_filter.yaw(), angle_filter.pitch(),
                    &pixel)) {
      return cv::Point2f(-1.f, -1.f);
    }
    return pixel;
  }

  // Association gate: the detection's disc centre must be close to the
  // predicted image position; the gate radius scales with the apparent target
  // size so fast motion / approaching targets are not rejected.
  bool associationOk(const Detection& detection) const {
    if (!angle_filter.initialized() || !detection.disc_ellipse.valid()) {
      return true;
    }
    const cv::Point2f predicted = predictedPixel();
    if (!(predicted.x >= 0.f && predicted.y >= 0.f)) {
      return true;  // no usable prediction: accept the detection
    }
    const double gate =
        std::max(kMinAssociationPx, kAssociationRadiusScale *
                                        predictedRadiusPx() + 8.0);
    const double dx = static_cast<double>(detection.disc_ellipse.center.x) -
                      static_cast<double>(predicted.x);
    const double dy = static_cast<double>(detection.disc_ellipse.center.y) -
                      static_cast<double>(predicted.y);
    return std::sqrt(dx * dx + dy * dy) <= gate;
  }

  // ROI for the detector: predicted centre +- (padding * radius), clamped to
  // the image. Empty rect == run the detector on the full frame.
  cv::Rect computeSearchRoi(const cv::Size& image_size) const {
    if (!config.detector.roi_search_enabled) {
      return cv::Rect();
    }
    cv::Point2f centre;
    if (!angle_filter.initialized() ||
        !projectLos(config.camera, angle_filter.yaw(), angle_filter.pitch(),
                    &centre)) {
      return cv::Rect();
    }
    const double radius = predictedRadiusPx();
    double padding = static_cast<double>(config.detector.roi_padding_scale) *
                     radius;
    padding = std::max(padding,
                       0.5 * static_cast<double>(config.tracker.roi_min_size_px));
    padding = std::max(padding,
                       0.5 * static_cast<double>(config.detector.roi_min_size_px));
    // Extra margin for unmodelled motion / prediction uncertainty.
    padding += 0.25 * radius;
    if (!std::isfinite(padding) || padding <= 0.0) {
      return cv::Rect();
    }
    const int half = static_cast<int>(std::lround(std::min(padding, 1.0e4)));
    if (half <= 0) {
      return cv::Rect();
    }
    cv::Rect roi(static_cast<int>(std::lround(centre.x)) - half,
                 static_cast<int>(std::lround(centre.y)) - half, 2 * half,
                 2 * half);
    roi &= cv::Rect(0, 0, image_size.width, image_size.height);
    if (roi.width < kMinRoiIntersectionPx ||
        roi.height < kMinRoiIntersectionPx) {
      return cv::Rect();
    }
    return roi;
  }

  void decayQuality() { quality = std::max(0.0, quality * decayPerFrame()); }

  // Forgets the current track (filters, pose continuity, ROI, counters).
  void stopTrack() {
    angle_filter.reset();
    range_filter.reset();
    pose_estimator.reset();
    last_pose = PoseEstimate();
    search_roi = cv::Rect();
    consecutive_hits = 0;
    consecutive_misses = 0;
    quality = 0.0;
    apparent_radius_px = 0.0;
  }

  // Runs the pose estimator and feeds the filters. Returns true when the
  // measurement was accepted by the angle filter.
  bool applyMeasurement(const PoseEstimate& pose, const Detection& detection) {
    if (!pose.valid || !(pose.distance_m > 0.0)) {
      return false;
    }
    const double radius =
        std::max(1.0f, detection.disc_ellipse.mean_radius());
    double sigma_scale =
        config.kalman.meas_sigma_scale_px / static_cast<double>(radius);
    if (!std::isfinite(sigma_scale)) {
      sigma_scale = 1.0;
    }
    sigma_scale = std::clamp(sigma_scale, 1.0, kMaxSigmaScale);
    const double base_sigma_deg =
        (std::isfinite(config.kalman.meas_sigma_deg) &&
         config.kalman.meas_sigma_deg > 0.0)
            ? config.kalman.meas_sigma_deg
            : 0.6;
    const double sigma_rad =
        std::max(1e-6, base_sigma_deg * kDeg2Rad * sigma_scale);

    // After coasting through a gap the filter covariance has grown and the
    // innovation gate may reject the re-detection even though the association
    // gate accepted it. Reacquire (re-seed the filters from the measurement)
    // instead of stalling until LOST.
    const bool coasting =
        (state == TrackState::Predicting) && consecutive_misses >= 1;
    bool reinitialized = false;

    bool angle_ok = true;
    if (!angle_filter.initialized()) {
      angle_filter.initialize(pose.yaw, pose.pitch);
    } else {
      angle_ok = angle_filter.update(pose.yaw, pose.pitch, sigma_rad);
      if (!angle_ok && coasting) {
        angle_filter.initialize(pose.yaw, pose.pitch);
        range_filter.initialize(pose.distance_m);
        reinitialized = true;
        angle_ok = true;
      }
    }
    if (!angle_ok) {
      return false;  // innovation gate rejected the measurement
    }

    const double range_sigma =
        (std::isfinite(config.kalman.range_meas_sigma_m) &&
         config.kalman.range_meas_sigma_m > 0.0)
            ? config.kalman.range_meas_sigma_m
            : 0.05;
    if (!range_filter.initialized()) {
      range_filter.initialize(pose.distance_m);
    } else if (!reinitialized) {
      range_filter.update(pose.distance_m, range_sigma);
    }

    apparent_radius_px = static_cast<double>(radius);
    double filter_quality = angle_filter.quality();
    if (!std::isfinite(filter_quality) || filter_quality <= 0.0) {
      filter_quality = 0.6;
    }
    if (reinitialized) {
      // A re-detection is trustworthy (association-gated PnP) but the filter
      // covariance was just reset; do not advertise a collapsed quality.
      filter_quality = std::max(filter_quality, 0.5);
    }
    quality = std::clamp(filter_quality, 0.05, 1.0);
    return true;
  }

  TrackOutput update(const cv::Mat& bgr, double timestamp_s, int frame_index) {
    ScopedTimer timer;
    TrackOutput output;
    output.frame_index = frame_index;
    output.timestamp_s = timestamp_s;
    output.state = state;
    output.track_id = -1;

    last_detection = Detection();
    last_detection.frame_index = frame_index;
    search_roi = cv::Rect();

    if (bgr.empty()) {
      last_processing_ms = timer.elapsedMs();
      output.state = state;
      return output;
    }

    // --- 1. time update ----------------------------------------------------
    double dt = 0.0;
    if (has_last_timestamp && timestamp_s > last_timestamp_s) {
      dt = timestamp_s - last_timestamp_s;
    }
    if (!std::isfinite(dt) || dt < 0.0) {
      dt = 0.0;
    }
    dt = std::min(dt, kMaxDtSeconds);
    last_timestamp_s = timestamp_s;
    has_last_timestamp = true;
    if (angle_filter.initialized()) {
      angle_filter.predict(dt);
    }
    if (range_filter.initialized()) {
      range_filter.predict(dt);
    }

    // --- 2. decide whether/how to search this frame ------------------------
    // SEARCH / IDLE: full-frame detection on EVERY frame (no prediction yet;
    // the full-frame detection is cheap enough -- ~2 ms at 720p). LOST is the
    // only state where the full-frame re-search is throttled by
    // full_search_interval.
    bool run_detection = true;
    cv::Rect roi;  // empty == full frame
    if (state == TrackState::Lost) {
      ++frames_since_full_search;
      const int interval = std::max(1, config.tracker.full_search_interval);
      if (frames_since_full_search >= interval) {
        frames_since_full_search = 0;
        run_detection = true;
      } else {
        run_detection = false;  // throttle full-frame searches while lost
      }
    } else if (state == TrackState::Tracking ||
               state == TrackState::Predicting ||
               state == TrackState::Acquiring) {
      roi = computeSearchRoi(bgr.size());
    }
    if (run_detection) {
      search_roi = roi;  // for the overlay / debug accessor
    }

    // --- 3. detect -> associate -> pose ------------------------------------
    bool measured = false;
    if (run_detection) {
      const Detection detection = detector.detect(bgr, roi, frame_index);
      last_detection = detection;
      if (detection.valid && associationOk(detection)) {
        const PoseEstimate pose = pose_estimator.estimate(detection);
        if (pose.valid) {
          last_pose = pose;
          measured = applyMeasurement(pose, detection);
        }
      }
    }

    // --- 4. state machine --------------------------------------------------
    if (measured) {
      ++consecutive_hits;
      consecutive_misses = 0;
      frames_since_full_search = 0;
      if (state == TrackState::Searching || state == TrackState::Idle ||
          state == TrackState::Lost) {
        state = (consecutive_hits >= acquireHits()) ? TrackState::Tracking
                                                    : TrackState::Acquiring;
      } else if (state == TrackState::Acquiring) {
        state = (consecutive_hits >= acquireHits()) ? TrackState::Tracking
                                                    : TrackState::Acquiring;
      } else {
        state = TrackState::Tracking;  // TRACK or PREDICT -> TRACK
      }
    } else {
      ++consecutive_misses;
      consecutive_hits = 0;
      switch (state) {
        case TrackState::Acquiring:
          // Not enough consecutive hits: restart the acquisition cleanly.
          stopTrack();
          state = TrackState::Searching;
          break;
        case TrackState::Tracking:
          state = TrackState::Predicting;
          decayQuality();
          break;
        case TrackState::Predicting:
          decayQuality();
          if (consecutive_misses >= maxMissFrames()) {
            stopTrack();
            state = TrackState::Lost;
            // Re-arm the throttled full-frame search immediately.
            frames_since_full_search =
                std::max(1, config.tracker.full_search_interval);
          }
          break;
        case TrackState::Lost:
          // Keep coasting in LOST: no valid output, full searches are
          // throttled by full_search_interval.
          break;
        case TrackState::Searching:
        case TrackState::Idle:
          state = TrackState::Searching;
          break;
      }
    }

    // --- 5. output ---------------------------------------------------------
    output.state = state;
    const bool filter_ready = angle_filter.initialized();
    const bool track_active =
        (state == TrackState::Tracking || state == TrackState::Predicting);
    output.valid = track_active && filter_ready && quality > 1e-4;
    output.measured = measured;
    output.predicted = (state == TrackState::Predicting);
    output.track_id = track_active ? 1 : -1;
    output.quality = output.valid ? std::clamp(quality, 0.0, 1.0) : 0.0;

    if (filter_ready) {
      double yaw = angle_filter.yaw();
      double pitch = angle_filter.pitch();
      double yaw_rate = angle_filter.yawRate();
      double pitch_rate = angle_filter.pitchRate();
      const double latency_s =
          (std::isfinite(config.kalman.latency_compensation_ms) &&
           config.kalman.latency_compensation_ms > 0.0)
              ? config.kalman.latency_compensation_ms / 1000.0
              : 0.0;
      if (latency_s > 0.0) {
        double predicted_yaw = yaw;
        double predicted_pitch = pitch;
        double predicted_yaw_rate = yaw_rate;
        double predicted_pitch_rate = pitch_rate;
        angle_filter.predictAhead(latency_s, &predicted_yaw, &predicted_pitch,
                                  &predicted_yaw_rate, &predicted_pitch_rate);
        yaw = predicted_yaw;
        pitch = predicted_pitch;
        yaw_rate = predicted_yaw_rate;
        pitch_rate = predicted_pitch_rate;
      }
      output.yaw = yaw;
      output.pitch = pitch;
      output.yaw_rate = yaw_rate;
      output.pitch_rate = pitch_rate;
      if (range_filter.initialized()) {
        output.distance_m = range_filter.range();
      } else if (last_pose.valid) {
        output.distance_m = last_pose.distance_m;
      }
    }

    last_processing_ms = timer.elapsedMs();
    return output;
  }

  void reset() {
    detector.reset();
    pose_estimator.reset();
    angle_filter.reset();
    range_filter.reset();
    state = TrackState::Searching;
    last_detection = Detection();
    last_pose = PoseEstimate();
    search_roi = cv::Rect();
    last_processing_ms = 0.0;
    has_last_timestamp = false;
    last_timestamp_s = 0.0;
    consecutive_hits = 0;
    consecutive_misses = 0;
    frames_since_full_search = 0;
    quality = 0.0;
    apparent_radius_px = 0.0;
  }
};

TargetTracker::TargetTracker(const RuntimeConfig& config)
    : impl_(new Impl(config)) {}

TargetTracker::~TargetTracker() = default;

TrackOutput TargetTracker::update(const cv::Mat& bgr, double timestamp_s,
                                  int frame_index) {
  return impl_->update(bgr, timestamp_s, frame_index);
}

void TargetTracker::reset() { impl_->reset(); }

const Detection& TargetTracker::lastDetection() const {
  return impl_->last_detection;
}

const PoseEstimate& TargetTracker::lastPose() const { return impl_->last_pose; }

cv::Rect TargetTracker::currentSearchRoi() const { return impl_->search_roi; }

TrackState TargetTracker::state() const { return impl_->state; }

std::string TargetTracker::stateName() const {
  return toString(impl_->state);
}

double TargetTracker::lastProcessingMs() const {
  return impl_->last_processing_ms;
}

const RuntimeConfig& TargetTracker::config() const { return impl_->config; }

}  // namespace glowtrack
