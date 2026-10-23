// TEMPORARY STUB -- owned by the detector workstream.
#include "glowtrack/detector.hpp"

#include <opencv2/imgproc.hpp>

namespace glowtrack {

struct TargetDetector::Impl {
  DetectionConfig config;
  TargetGeometry target;
  cv::Mat debug_mask;
  cv::Mat debug_candidates;
};

TargetDetector::TargetDetector(const DetectionConfig& detector_config,
                               const TargetGeometry& target)
    : impl_(new Impl()) {
  impl_->config = detector_config;
  impl_->target = target;
}

TargetDetector::~TargetDetector() = default;

Detection TargetDetector::detect(const cv::Mat& bgr, const cv::Rect& search_roi,
                                 int frame_index) {
  (void)bgr;
  (void)search_roi;
  Detection detection;
  detection.frame_index = frame_index;
  impl_->debug_mask = cv::Mat();
  impl_->debug_candidates = cv::Mat();
  return detection;
}

void TargetDetector::reset() {}

const cv::Mat& TargetDetector::debugMask() const { return impl_->debug_mask; }
const cv::Mat& TargetDetector::debugCandidates() const {
  return impl_->debug_candidates;
}

}  // namespace glowtrack
