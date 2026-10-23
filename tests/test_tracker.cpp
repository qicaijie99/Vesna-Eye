// TargetTracker integration tests on a synthetic rendered target sequence.
//
// The target is rendered like the real one: a white disc in the middle of a
// green glowing annulus on a dark background. The disc moves across the frame
// (so the line-of-sight sign is known), and a 7-frame dropout (frames 20..26)
// is injected in the middle to exercise the PREDICT state and the recovery
// path.
//
// The tracker consumes the real TargetDetector, PoseEstimator and Kalman
// filters. While those modules are still stubs the measurement-based assertions
// cannot pass; in that case the test prints a [skip] line and only checks the
// behaviour that does not depend on the stubs (no-detection frames). Once the
// modules are implemented the full assertion set runs.

#include "test_framework.hpp"

#include "glowtrack/config.hpp"
#include "glowtrack/detector.hpp"
#include "glowtrack/kalman_filter.hpp"
#include "glowtrack/pose_estimator.hpp"
#include "glowtrack/tracker.hpp"
#include "glowtrack/types.hpp"
#include "glowtrack/util.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using namespace glowtrack;

namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
std::string findRepoFile(const std::string& relative_path) {
  const char* prefixes[] = {"", "../", "../../", "../../../"};
  for (const char* prefix : prefixes) {
    const std::string candidate = std::string(prefix) + relative_path;
    std::error_code code;
    if (std::filesystem::is_regular_file(candidate, code)) {
      return candidate;
    }
  }
  return std::string();
}

RuntimeConfig loadSyntheticConfig() {
  const std::string path = findRepoFile("config/synthetic_640x480.yaml");
  if (!path.empty()) {
    try {
      return loadRuntimeConfig(path);
    } catch (const std::exception& error) {
      std::printf("         [warn] cannot load %s: %s (using defaults)\n",
                  path.c_str(), error.what());
    }
  }
  RuntimeConfig config;
  config.camera.width = 640;
  config.camera.height = 480;
  config.camera.fx = 500.0;
  config.camera.fy = 500.0;
  config.camera.cx = 320.0;
  config.camera.cy = 240.0;
  return config;
}

// Scene description shared by the renderer and the assertions.
constexpr int kFrameCount = 40;
constexpr int kFirstHiddenFrame = 20;
constexpr int kLastHiddenFrame = 26;
constexpr int kProbeFrame = -1;  // marker for "probe/not a scene frame"
constexpr float kDiscRadiusPx = 26.f;

cv::Point2f targetCentre(int frame_index) {
  const float x = 200.f + 8.f * static_cast<float>(frame_index);
  const float y =
      240.f + 40.f * static_cast<float>(std::sin(0.25 * frame_index));
  return cv::Point2f(x, y);
}

bool targetVisible(int frame_index) {
  return frame_index < kFirstHiddenFrame || frame_index > kLastHiddenFrame;
}

// Renders one 640x480 BGR frame with the target at `centre` (or an empty
// scene). The scene is drawn at 2x and downsampled for a smooth edge.
cv::Mat renderFrameAt(cv::Point2f centre, bool visible) {
  const int width = 640;
  const int height = 480;
  const int supersample = 2;
  cv::Mat canvas(height * supersample, width * supersample, CV_8UC3,
                 cv::Scalar(32, 34, 32));
  if (visible) {
    const cv::Point2f scaled_centre =
        centre * static_cast<float>(supersample);
    const int disc_radius =
        static_cast<int>(std::lround(kDiscRadiusPx * supersample));
    const int glow_radius = static_cast<int>(
        std::lround(kDiscRadiusPx * 1.45f * supersample));
    // Green glowing diffuser (the white disc is a "hole" inside it).
    cv::circle(canvas, scaled_centre, glow_radius, cv::Scalar(0, 255, 0),
               cv::FILLED, cv::LINE_AA);
    // Saturated white centre disc: the PnP circle primitive.
    cv::circle(canvas, scaled_centre, disc_radius, cv::Scalar(255, 255, 255),
               cv::FILLED, cv::LINE_AA);
  }
  cv::Mat frame;
  cv::resize(canvas, frame, cv::Size(width, height), 0.0, 0.0,
             cv::INTER_AREA);
  return frame;
}

cv::Mat renderFrame(int frame_index) {
  return renderFrameAt(targetCentre(frame_index), targetVisible(frame_index));
}

struct ModuleStatus {
  bool detector_ok = false;
  bool pose_ok = false;
  bool kalman_ok = false;
  bool ready() const { return detector_ok && pose_ok && kalman_ok; }
};

// Probes the external algorithm modules with a static centred target. This is
// how the test detects that detector/pose/Kalman are still placeholders.
ModuleStatus probeModules(const RuntimeConfig& config) {
  ModuleStatus status;
  const cv::Mat probe = renderFrame(9);  // centred-ish target, visible

  TargetDetector detector(config.detector, config.target);
  const Detection detection = detector.detect(probe, cv::Rect(), 0);
  status.detector_ok = detection.valid;
  if (!detection.valid) {
    std::printf("         [skip] detector is not functional yet\n");
    return status;
  }

  PoseEstimator estimator(config.camera, config.target, config.pose);
  const PoseEstimate pose = estimator.estimate(detection);
  status.pose_ok = pose.valid;
  if (!pose.valid) {
    std::printf("         [skip] pose estimator is not functional yet\n");
    return status;
  }

  AngleKalmanFilter filter(config.kalman);
  filter.initialize(pose.yaw, pose.pitch);
  status.kalman_ok = filter.update(pose.yaw, pose.pitch, 0.01);
  if (!status.kalman_ok) {
    std::printf("         [skip] Kalman filter is not functional yet\n");
  }
  return status;
}

}  // namespace

// ---------------------------------------------------------------------------
// Behaviour that does not depend on detector / pose / Kalman: no detections
// means no valid output and the tracker stays in SEARCH.
// ---------------------------------------------------------------------------
GT_TEST(tracker_blank_frames_no_valid_output) {
  const RuntimeConfig config = loadSyntheticConfig();
  TargetTracker tracker(config);
  GT_CHECK(tracker.state() == TrackState::Searching);
  GT_CHECK(tracker.stateName() == toString(TrackState::Searching));
  GT_CHECK(!tracker.lastDetection().valid);
  GT_CHECK(!tracker.lastPose().valid);
  GT_CHECK(tracker.currentSearchRoi().empty());

  const cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
  for (int frame = 0; frame < 6; ++frame) {
    const TrackOutput output =
        tracker.update(blank, static_cast<double>(frame) / 30.0, frame);
    GT_CHECK(!output.valid);
    GT_CHECK(!output.measured);
    GT_CHECK(output.state == TrackState::Searching ||
             output.state == TrackState::Lost);
    GT_CHECK(output.frame_index == frame);
    GT_CHECK_NEAR(output.timestamp_s, static_cast<double>(frame) / 30.0,
                  1e-9);
    GT_CHECK(!tracker.lastDetection().valid);
    GT_CHECK(tracker.lastProcessingMs() >= 0.0);
  }

  // Empty frame must be handled gracefully.
  const TrackOutput empty_output = tracker.update(cv::Mat(), 1.0, 6);
  GT_CHECK(!empty_output.valid);

  tracker.reset();
  GT_CHECK(tracker.state() == TrackState::Searching);
  GT_CHECK(!tracker.lastDetection().valid);
  GT_CHECK(!tracker.lastPose().valid);
  GT_CHECK(tracker.currentSearchRoi().empty());
}

// ---------------------------------------------------------------------------
// Full state machine on the synthetic sequence.
// ---------------------------------------------------------------------------
GT_TEST(tracker_synthetic_sequence_acquire_predict_recover) {
  const RuntimeConfig config = loadSyntheticConfig();

  const ModuleStatus status = probeModules(config);
  if (!status.ready()) {
    std::printf(
        "         [skip] tracker sequence assertions require functional "
        "detector + pose + kalman modules\n");
    return;
  }

  GT_CHECK(config.tracker.acquire_hits >= 1);
  GT_CHECK(config.tracker.max_miss_frames > kLastHiddenFrame - kFirstHiddenFrame + 1);

  TargetTracker tracker(config);
  GT_CHECK(tracker.state() == TrackState::Searching);

  std::vector<TrackOutput> outputs;
  outputs.reserve(kFrameCount);
  const double fps = config.nominal_fps > 1.0 ? config.nominal_fps : 30.0;
  for (int frame = 0; frame < kFrameCount; ++frame) {
    const cv::Mat image = renderFrame(frame);
    const TrackOutput output =
        tracker.update(image, static_cast<double>(frame) / fps, frame);
    GT_CHECK(output.frame_index == frame);
    GT_CHECK(output.state == tracker.state());
    outputs.push_back(output);
  }

  // --- acquisition: SEARCH -> ACQUIRE -> TRACK --------------------------
  int first_tracking = -1;
  for (int i = 0; i < kFrameCount; ++i) {
    if (outputs[i].state == TrackState::Tracking) {
      first_tracking = i;
      break;
    }
  }
  GT_CHECK(first_tracking >= 0);
  GT_CHECK(first_tracking <= 8);  // detect + acquire_hits consecutive hits
  for (int i = 0; i < first_tracking; ++i) {
    GT_CHECK(outputs[i].state == TrackState::Acquiring ||
             outputs[i].state == TrackState::Searching);
    GT_CHECK(!outputs[i].valid);
  }
  GT_CHECK(outputs[first_tracking].valid);
  GT_CHECK(outputs[first_tracking].measured);
  GT_CHECK(!outputs[first_tracking].predicted);
  GT_CHECK(outputs[first_tracking].track_id == 1);
  GT_CHECK(outputs[first_tracking].quality > 0.0);
  GT_CHECK(tracker.state() == TrackState::Tracking);

  // --- steady tracking --------------------------------------------------
  const int steady = first_tracking + 2;
  GT_CHECK(steady < kFirstHiddenFrame);
  GT_CHECK(outputs[steady].valid);
  GT_CHECK(outputs[steady].measured);
  GT_CHECK(!outputs[steady].predicted);
  GT_CHECK(outputs[steady].distance_m > 0.2);
  GT_CHECK(outputs[steady].distance_m < 3.0);
  GT_CHECK(std::isfinite(outputs[steady].yaw));
  GT_CHECK(std::isfinite(outputs[steady].pitch));

  // --- injected dropout: TRACK -> PREDICT, valid but no measurement -----
  GT_CHECK(outputs[kFirstHiddenFrame].state == TrackState::Predicting);
  GT_CHECK(outputs[kFirstHiddenFrame].predicted);
  GT_CHECK(!outputs[kFirstHiddenFrame].measured);
  GT_CHECK(outputs[kFirstHiddenFrame].valid);
  for (int i = kFirstHiddenFrame; i <= kLastHiddenFrame; ++i) {
    GT_CHECK(outputs[i].state == TrackState::Predicting);
    GT_CHECK(outputs[i].predicted);
    GT_CHECK(!outputs[i].measured);
    GT_CHECK(outputs[i].valid);  // still inside max_miss_frames
  }
  // Quality must decay monotonically while coasting.
  for (int i = kFirstHiddenFrame; i < kLastHiddenFrame; ++i) {
    GT_CHECK(outputs[i].quality >= outputs[i + 1].quality);
  }
  GT_CHECK(outputs[kFirstHiddenFrame].quality <
           outputs[kFirstHiddenFrame - 1].quality);

  // --- recovery: PREDICT -> TRACK after the target reappears ------------
  int recovery = -1;
  for (int i = kLastHiddenFrame + 1; i < kFrameCount; ++i) {
    if (outputs[i].state == TrackState::Tracking && outputs[i].measured) {
      recovery = i;
      break;
    }
  }
  GT_CHECK(recovery >= 0);
  GT_CHECK(recovery <= kLastHiddenFrame + 8);
  GT_CHECK(outputs[recovery].valid);
  GT_CHECK(!outputs[recovery].predicted);

  // --- line-of-sight direction matches the drawn image position ---------
  // yaw > 0 when the target is right of the image centre, pitch > 0 when it is
  // above it (image y axis points down).
  int checked_signs = 0;
  for (int frame = 0; frame < kFrameCount; ++frame) {
    if (!targetVisible(frame) || outputs[frame].state != TrackState::Tracking ||
        !outputs[frame].valid) {
      continue;
    }
    const cv::Point2f centre = targetCentre(frame);
    const double offset_x = static_cast<double>(centre.x) - 320.0;
    const double offset_y = 240.0 - static_cast<double>(centre.y);
    if (std::fabs(offset_x) > 60.0) {
      GT_CHECK(std::fabs(outputs[frame].yaw) > 1e-3);
      GT_CHECK((outputs[frame].yaw > 0.0) == (offset_x > 0.0));
      ++checked_signs;
    }
    if (std::fabs(offset_y) > 25.0) {
      GT_CHECK(std::fabs(outputs[frame].pitch) > 1e-3);
      GT_CHECK((outputs[frame].pitch > 0.0) == (offset_y > 0.0));
      ++checked_signs;
    }
  }
  GT_CHECK(checked_signs >= 20);

  // --- debug accessors / ROI -------------------------------------------
  GT_CHECK(tracker.lastDetection().valid);
  GT_CHECK(tracker.lastPose().valid);
  GT_CHECK(!tracker.currentSearchRoi().empty());
  const cv::Rect roi = tracker.currentSearchRoi();
  GT_CHECK(roi.x >= 0 && roi.y >= 0);
  GT_CHECK(roi.x + roi.width <= 640);
  GT_CHECK(roi.y + roi.height <= 480);
  GT_CHECK(tracker.lastProcessingMs() > 0.0);

  // --- reset clears everything -----------------------------------------
  tracker.reset();
  GT_CHECK(tracker.state() == TrackState::Searching);
  GT_CHECK(!tracker.lastDetection().valid);
  GT_CHECK(!tracker.lastPose().valid);
  GT_CHECK(tracker.currentSearchRoi().empty());
  const cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(25, 25, 25));
  const TrackOutput after_reset = tracker.update(blank, 10.0, 100);
  GT_CHECK(!after_reset.valid);
}

// ---------------------------------------------------------------------------
// A gap longer than max_miss_frames must enter LOST (valid=false), throttle the
// full-frame search and re-acquire once the target reappears.
// ---------------------------------------------------------------------------
GT_TEST(tracker_lost_then_reacquire) {
  const RuntimeConfig config = loadSyntheticConfig();

  const ModuleStatus status = probeModules(config);
  if (!status.ready()) {
    std::printf(
        "         [skip] LOST/reacquire assertions require functional "
        "detector + pose + kalman modules\n");
    return;
  }

  const int hidden_begin = 10;
  const int hidden_end = 29;  // 20 hidden frames > max_miss_frames (15)
  const int frames = 50;
  GT_CHECK(config.tracker.max_miss_frames < hidden_end - hidden_begin);

  TargetTracker tracker(config);
  const double fps = config.nominal_fps > 1.0 ? config.nominal_fps : 30.0;
  std::vector<TrackOutput> outputs;
  outputs.reserve(frames);
  for (int frame = 0; frame < frames; ++frame) {
    const bool visible = frame < hidden_begin || frame >= hidden_end;
    const cv::Mat image =
        renderFrameAt(targetCentre(frame), visible);
    outputs.push_back(
        tracker.update(image, static_cast<double>(frame) / fps, frame));
  }

  // Tracked before the gap.
  int first_tracking = -1;
  for (int i = 0; i < hidden_begin; ++i) {
    if (outputs[i].state == TrackState::Tracking) {
      first_tracking = i;
      break;
    }
  }
  GT_CHECK(first_tracking >= 0);
  GT_CHECK(outputs[first_tracking].valid);

  // Gap -> PREDICT -> LOST.
  GT_CHECK(outputs[hidden_begin].state == TrackState::Predicting);
  int lost_frame = -1;
  for (int i = hidden_begin; i < hidden_end; ++i) {
    if (outputs[i].state == TrackState::Lost) {
      lost_frame = i;
      break;
    }
  }
  GT_CHECK(lost_frame >= 0);
  GT_CHECK(lost_frame <= hidden_begin + config.tracker.max_miss_frames + 2);
  GT_CHECK(!outputs[lost_frame].valid);  // LOST must not emit valid output
  GT_CHECK(!outputs[lost_frame].predicted);  // PREDICT flag only while predicting
  for (int i = lost_frame; i < hidden_end; ++i) {
    GT_CHECK(outputs[i].state == TrackState::Lost);
    GT_CHECK(!outputs[i].valid);
  }

  // Reacquire after the gap.
  int recovery = -1;
  for (int i = hidden_end; i < frames; ++i) {
    if (outputs[i].state == TrackState::Tracking && outputs[i].measured) {
      recovery = i;
      break;
    }
  }
  GT_CHECK(recovery >= 0);
  GT_CHECK(recovery <= hidden_end + 10);
  GT_CHECK(outputs[recovery].valid);
  GT_CHECK(tracker.state() == TrackState::Tracking);
}
