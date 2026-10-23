// TEMPORARY STUB -- owned by the pipeline/IO workstream.
#include "glowtrack/tracker.hpp"

#include <opencv2/core.hpp>

namespace glowtrack {

struct TargetTracker::Impl {
  RuntimeConfig config;
  Detection last_detection;
  PoseEstimate last_pose;
  cv::Rect search_roi;
  TrackState state = TrackState::Searching;
  double last_processing_ms = 0.0;
};

TargetTracker::TargetTracker(const RuntimeConfig& config) : impl_(new Impl()) {
  impl_->config = config;
}

TargetTracker::~TargetTracker() = default;

TrackOutput TargetTracker::update(const cv::Mat& bgr, double timestamp_s,
                                  int frame_index) {
  (void)bgr;
  TrackOutput output;
  output.frame_index = frame_index;
  output.timestamp_s = timestamp_s;
  output.state = impl_->state;
  output.valid = false;
  return output;
}

void TargetTracker::reset() {
  impl_->last_detection = Detection();
  impl_->last_pose = PoseEstimate();
  impl_->state = TrackState::Searching;
}

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
