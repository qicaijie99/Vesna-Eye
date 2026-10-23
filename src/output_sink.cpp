// TEMPORARY STUB -- owned by the pipeline/IO workstream.
#include "glowtrack/output_sink.hpp"
#include "glowtrack/util.hpp"

#include <opencv2/imgproc.hpp>

#include <cstdio>

namespace glowtrack {

struct CsvLogger::Impl {
  std::string path;
  std::string error;
  bool ok = false;
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
  }
}

bool CsvLogger::ok() const { return impl_->ok; }
const std::string& CsvLogger::path() const { return impl_->path; }
const std::string& CsvLogger::lastError() const { return impl_->error; }

void CsvLogger::writeHeader() {
  if (!impl_->ok) {
    return;
  }
  std::fprintf(impl_->file,
               "frame,timestamp_s,state,detected,valid,measured,predicted,"
               "yaw_deg,pitch_deg,yaw_rate_dps,pitch_rate_dps,distance_m,"
               "quality,reproj_rmse_px,track_id,raw_yaw_deg,raw_pitch_deg,"
               "ellipse_cx,ellipse_cy,semi_major,semi_minor,ellipse_angle_deg\n");
}

void CsvLogger::log(const TrackOutput& output, const Detection& detection,
                    const PoseEstimate& pose) {
  if (!impl_->ok) {
    return;
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
}

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
                                const cv::Rect& search_roi,
                                double processing_ms, double fps) {
  (void)output;
  (void)detection;
  (void)pose;
  (void)search_roi;
  (void)processing_ms;
  (void)fps;
  return bgr.clone();
}

OverlayVideoWriter::OverlayVideoWriter(const std::string& path, double fps,
                                       int width, int height)
    : path_(path) {
  if (!ensureParentDirectory(path)) {
    error_ = "cannot create parent directory for " + path;
    return;
  }
  ok_ = writer_.open(path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps,
                     cv::Size(width, height));
  if (!ok_) {
    error_ = "cannot open video writer for " + path;
  }
}

OverlayVideoWriter::~OverlayVideoWriter() = default;

bool OverlayVideoWriter::ok() const { return ok_; }
const std::string& OverlayVideoWriter::lastError() const { return error_; }

void OverlayVideoWriter::write(const cv::Mat& frame) {
  if (ok_) {
    writer_.write(frame);
  }
}

}  // namespace glowtrack
