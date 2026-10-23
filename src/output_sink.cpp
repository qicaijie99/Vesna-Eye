// Outputs: CSV log, annotated overlay video renderer/writer, serial packet.
//
// CSV schema (must stay stable -- tools/eval_metrics.cpp parses it):
//   frame,timestamp_s,state,detected,valid,measured,predicted,
//   yaw_deg,pitch_deg,yaw_rate_dps,pitch_rate_dps,distance_m,quality,
//   reproj_rmse_px,track_id,raw_yaw_deg,raw_pitch_deg,
//   ellipse_cx,ellipse_cy,semi_major,semi_minor,ellipse_angle_deg

#include "glowtrack/output_sink.hpp"

#include "glowtrack/util.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace glowtrack {
namespace {

// ---------------------------------------------------------------------------
// Overlay drawing helpers
// ---------------------------------------------------------------------------
cv::Scalar stateColor(TrackState state, bool valid) {
  switch (state) {
    case TrackState::Tracking:
      return valid ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 160, 160);
    case TrackState::Predicting:
      return cv::Scalar(0, 220, 255);  // yellow
    case TrackState::Acquiring:
      return cv::Scalar(0, 160, 255);  // orange
    case TrackState::Lost:
      return cv::Scalar(0, 0, 255);  // red
    case TrackState::Searching:
      return cv::Scalar(220, 220, 220);
    case TrackState::Idle:
      break;
  }
  return cv::Scalar(200, 200, 200);
}

// Project a line-of-sight direction given as tracker angles into the image.
cv::Point2f projectLos(const CameraIntrinsics& camera, double yaw,
                       double pitch) {
  const double z = 1.0;
  const double x = std::tan(yaw);
  const double y = -std::tan(pitch);
  std::vector<cv::Point3d> object_points;
  object_points.emplace_back(x, y, z);
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(object_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                    camera.cameraMatrix(), camera.distortion(), image_points);
  if (image_points.empty()) {
    return cv::Point2f(-1.f, -1.f);
  }
  return cv::Point2f(static_cast<float>(image_points[0].x),
                     static_cast<float>(image_points[0].y));
}

void drawEllipseAxes(cv::Mat& canvas, const Ellipse2D& ellipse,
                     const cv::Scalar& color) {
  if (!ellipse.valid()) {
    return;
  }
  const double angle = static_cast<double>(ellipse.angle_deg) * kDeg2Rad;
  const cv::Point2f major_dir(static_cast<float>(std::cos(angle)),
                              static_cast<float>(std::sin(angle)));
  const cv::Point2f minor_dir(-major_dir.y, major_dir.x);
  cv::line(canvas, ellipse.center + major_dir * ellipse.semi_major,
           ellipse.center - major_dir * ellipse.semi_major, color, 1,
           cv::LINE_AA);
  cv::line(canvas, ellipse.center + minor_dir * ellipse.semi_minor,
           ellipse.center - minor_dir * ellipse.semi_minor, color, 1,
           cv::LINE_AA);
}

void drawEllipseShape(cv::Mat& canvas, const Ellipse2D& ellipse,
                      const cv::Scalar& color) {
  if (!ellipse.valid()) {
    return;
  }
  const int semi_major = std::max(1, static_cast<int>(std::lround(ellipse.semi_major)));
  const int semi_minor = std::max(1, static_cast<int>(std::lround(ellipse.semi_minor)));
  cv::ellipse(canvas, ellipse.center, cv::Size(semi_major, semi_minor),
              static_cast<double>(ellipse.angle_deg), 0.0, 360.0, color, 2,
              cv::LINE_AA);
}

void drawPnpAxes(cv::Mat& canvas, const RuntimeConfig& config,
                 const PoseEstimate& pose) {
  if (!pose.valid) {
    return;
  }
  // Axes of the target frame: origin at the white disc centre, +-50 mm.
  std::vector<cv::Point3f> object_points{
      cv::Point3f(0.f, 0.f, 0.f), cv::Point3f(50.f, 0.f, 0.f),
      cv::Point3f(0.f, 50.f, 0.f), cv::Point3f(0.f, 0.f, 50.f)};
  std::vector<cv::Point2f> image_points;
  try {
    cv::projectPoints(object_points, pose.rvec, pose.tvec,
                      config.camera.cameraMatrix(), config.camera.distortion(),
                      image_points);
  } catch (const cv::Exception&) {
    return;
  }
  if (image_points.size() != object_points.size()) {
    return;
  }
  cv::line(canvas, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 2,
           cv::LINE_AA);  // X axis: red
  cv::line(canvas, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 2,
           cv::LINE_AA);  // Y axis: green
  cv::line(canvas, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 2,
           cv::LINE_AA);  // Z axis: blue
}

void drawHud(cv::Mat& canvas, const RuntimeConfig& config,
             const TrackOutput& output, const Detection& detection,
             const PoseEstimate& pose, double processing_ms, double fps) {
  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const double font_scale = 0.45;
  const int thickness = 1;
  const int line_height = 17;
  const int padding = 8;

  std::vector<std::string> lines;
  lines.reserve(8);
  lines.push_back(formatString("state: %-7s frame: %d",
                               toString(output.state), output.frame_index));
  lines.push_back(formatString("yaw %+7.2f deg   pitch %+7.2f deg",
                               output.yaw * kRad2Deg, output.pitch * kRad2Deg));
  lines.push_back(formatString("yaw_rate %+7.1f dps  pitch_rate %+7.1f dps",
                               output.yaw_rate * kRad2Deg,
                               output.pitch_rate * kRad2Deg));
  lines.push_back(formatString("range %.2f m   quality %.2f",
                               output.distance_m, output.quality));
  lines.push_back(formatString("fps %.1f   proc %.2f ms", fps, processing_ms));
  lines.push_back(formatString("valid %d  meas %d  pred %d  det %d",
                               output.valid ? 1 : 0, output.measured ? 1 : 0,
                               output.predicted ? 1 : 0,
                               detection.valid ? 1 : 0));
  if (pose.valid) {
    lines.push_back(formatString("reproj %.2f px  track_id %d",
                                 pose.reproj_rmse_px, output.track_id));
  } else {
    lines.push_back(formatString("reproj --     track_id %d",
                                 output.track_id));
  }

  int text_width = 0;
  for (const auto& line : lines) {
    int baseline = 0;
    const cv::Size size =
        cv::getTextSize(line, font, font_scale, thickness, &baseline);
    text_width = std::max(text_width, size.width);
  }
  const int box_width = text_width + 2 * padding;
  const int box_height =
      static_cast<int>(lines.size()) * line_height + 2 * padding - 4;
  cv::Rect box(6, 6, box_width, box_height);
  box &= cv::Rect(0, 0, canvas.cols, canvas.rows);
  if (box.width > 0 && box.height > 0) {
    cv::Mat region = canvas(box);
    const cv::Mat dimmed(box.size(), region.type(), cv::Scalar(0, 0, 0));
    cv::addWeighted(region, 0.30, dimmed, 0.70, 0.0, region);
  }

  const cv::Scalar accent = stateColor(output.state, output.valid);
  int y = 6 + padding + 12;
  for (size_t i = 0; i < lines.size(); ++i) {
    const cv::Scalar color = (i == 0) ? accent : cv::Scalar(230, 230, 230);
    cv::putText(canvas, lines[i], cv::Point(6 + padding, y), font, font_scale,
                color, thickness, cv::LINE_AA);
    y += line_height;
  }
  // Keep the compiler honest about unused configuration (reserved for future
  // HUD options, e.g. hiding fields based on output config).
  (void)config;
}

}  // namespace

// ===========================================================================
// CsvLogger
// ===========================================================================
struct CsvLogger::Impl {
  std::string path;
  std::string error;
  bool ok = false;
  bool header_written = false;
  FILE* file = nullptr;
};

CsvLogger::CsvLogger(const std::string& path) : impl_(new Impl()) {
  impl_->path = path;
  if (!ensureParentDirectory(path)) {
    impl_->error = "cannot create parent directory for " + path;
    return;
  }
  impl_->file = std::fopen(path.c_str(), "w");
  impl_->ok = impl_->file != nullptr;
  if (!impl_->ok) {
    impl_->error = "cannot open " + path;
  }
}

CsvLogger::~CsvLogger() {
  if (impl_->file != nullptr) {
    std::fclose(impl_->file);
    impl_->file = nullptr;
  }
}

bool CsvLogger::ok() const { return impl_->ok; }
const std::string& CsvLogger::path() const { return impl_->path; }
const std::string& CsvLogger::lastError() const { return impl_->error; }

void CsvLogger::writeHeader() {
  if (!impl_->ok || impl_->header_written) {
    return;
  }
  std::fprintf(
      impl_->file,
      "frame,timestamp_s,state,detected,valid,measured,predicted,"
      "yaw_deg,pitch_deg,yaw_rate_dps,pitch_rate_dps,distance_m,"
      "quality,reproj_rmse_px,track_id,raw_yaw_deg,raw_pitch_deg,"
      "ellipse_cx,ellipse_cy,semi_major,semi_minor,ellipse_angle_deg\n");
  std::fflush(impl_->file);
  impl_->header_written = true;
}

void CsvLogger::log(const TrackOutput& output, const Detection& detection,
                    const PoseEstimate& pose) {
  if (!impl_->ok) {
    return;
  }
  if (!impl_->header_written) {
    writeHeader();
  }
  std::fprintf(impl_->file,
               "%d,%.6f,%s,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%d,"
               "%.6f,%.6f,%.2f,%.2f,%.3f,%.3f,%.3f\n",
               output.frame_index, output.timestamp_s, toString(output.state),
               detection.valid ? 1 : 0, output.valid ? 1 : 0,
               output.measured ? 1 : 0, output.predicted ? 1 : 0,
               output.yaw * kRad2Deg, output.pitch * kRad2Deg,
               output.yaw_rate * kRad2Deg, output.pitch_rate * kRad2Deg,
               output.distance_m, output.quality, pose.reproj_rmse_px,
               output.track_id, pose.valid ? pose.yaw * kRad2Deg : 0.0,
               pose.valid ? pose.pitch * kRad2Deg : 0.0,
               detection.valid ? detection.disc_ellipse.center.x : 0.0,
               detection.valid ? detection.disc_ellipse.center.y : 0.0,
               detection.valid ? detection.disc_ellipse.semi_major : 0.0,
               detection.valid ? detection.disc_ellipse.semi_minor : 0.0,
               detection.valid ? detection.disc_ellipse.angle_deg : 0.0);
  std::fflush(impl_->file);  // keep the log readable by external tools/scripts
}

// ===========================================================================
// OverlayRenderer
// ===========================================================================
struct OverlayRenderer::Impl {
  RuntimeConfig config;
};

OverlayRenderer::OverlayRenderer(const RuntimeConfig& config)
    : impl_(new Impl()) {
  impl_->config = config;
}

OverlayRenderer::~OverlayRenderer() = default;

cv::Mat OverlayRenderer::render(const cv::Mat& bgr, const TrackOutput& output,
                                const Detection& detection,
                                const PoseEstimate& pose,
                                const cv::Rect& search_roi, double processing_ms,
                                double fps) {
  if (bgr.empty()) {
    return bgr.clone();
  }
  cv::Mat canvas = bgr.clone();
  const cv::Scalar roi_color(255, 128, 0);    // blue (BGR)
  const cv::Scalar disc_color(0, 0, 255);     // red
  const cv::Scalar glow_color(0, 255, 0);     // green
  const cv::Scalar los_color(255, 0, 255);    // magenta

  // Search ROI actually used by the detector this frame (empty = full frame).
  if (!search_roi.empty()) {
    const cv::Rect clipped = search_roi & cv::Rect(0, 0, canvas.cols, canvas.rows);
    if (!clipped.empty()) {
      cv::rectangle(canvas, clipped, roi_color, 1, cv::LINE_AA);
      cv::putText(canvas, "ROI", cv::Point(clipped.x + 3, clipped.y + 13),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, roi_color, 1, cv::LINE_AA);
    }
  }

  // Detected glow (green outline) and geometric disc (red outline).
  if (detection.valid) {
    drawEllipseShape(canvas, detection.glow_ellipse, glow_color);
    drawEllipseShape(canvas, detection.disc_ellipse, disc_color);
    drawEllipseAxes(canvas, detection.disc_ellipse, cv::Scalar(0, 128, 255));
    if (detection.disc_ellipse.valid()) {
      cv::circle(canvas, detection.disc_ellipse.center, 2, disc_color,
                 cv::FILLED, cv::LINE_AA);
    }
  }

  // PnP target axes (optional, only with a valid pose).
  drawPnpAxes(canvas, impl_->config, pose);

  // Image centre crosshair.
  const cv::Point centre(canvas.cols / 2, canvas.rows / 2);
  cv::drawMarker(canvas, centre, cv::Scalar(0, 220, 220), cv::MARKER_CROSS, 18,
                 1, cv::LINE_AA);

  // Predicted line of sight (from the filtered / compensated output angles).
  if (output.valid && std::fabs(output.yaw) < 1.4 &&
      std::fabs(output.pitch) < 1.4) {
    const cv::Point2f los =
        projectLos(impl_->config.camera, output.yaw, output.pitch);
    if (std::isfinite(los.x) && std::isfinite(los.y) &&
        los.x >= -1.f && los.y >= -1.f && los.x <= canvas.cols + 1 &&
        los.y <= canvas.rows + 1) {
      cv::circle(canvas, los, 6, los_color, 1, cv::LINE_AA);
      cv::drawMarker(canvas, los, los_color, cv::MARKER_CROSS, 12, 1,
                     cv::LINE_AA);
    }
  }

  drawHud(canvas, impl_->config, output, detection, pose, processing_ms, fps);
  return canvas;
}

// ===========================================================================
// OverlayVideoWriter
// ===========================================================================
OverlayVideoWriter::OverlayVideoWriter(const std::string& path, double fps,
                                       int width, int height)
    : path_(path) {
  if (width <= 0 || height <= 0) {
    error_ = "invalid overlay frame size";
    return;
  }
  if (!ensureParentDirectory(path)) {
    error_ = "cannot create parent directory for " + path;
    return;
  }
  const double use_fps =
      (std::isfinite(fps) && fps > 0.5 && fps < 1000.0) ? fps : 30.0;
  const cv::Size frame_size(width, height);
  ok_ = writer_.open(path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), use_fps,
                     frame_size);
  if (!ok_) {
    ok_ = writer_.open(path_, cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
                       use_fps, frame_size);
  }
  if (!ok_) {
    // Generic fallback: the extension may not be honoured by every backend,
    // but MJPG is supported by the default FFmpeg/MSMF writers.
    ok_ = writer_.open(path_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                       use_fps, frame_size);
  }
  if (!ok_) {
    error_ = "cannot open video writer for " + path;
  }
}

OverlayVideoWriter::~OverlayVideoWriter() = default;

bool OverlayVideoWriter::ok() const { return ok_; }
const std::string& OverlayVideoWriter::lastError() const { return error_; }

void OverlayVideoWriter::write(const cv::Mat& frame) {
  if (ok_ && !frame.empty()) {
    writer_.write(frame);
  }
}

}  // namespace glowtrack
