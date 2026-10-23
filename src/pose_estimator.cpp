// TEMPORARY STUB -- owned by the pose/filter workstream.
#include "glowtrack/pose_estimator.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace glowtrack {

struct PoseEstimator::Impl {
  CameraIntrinsics camera;
  TargetGeometry target;
  PoseConfig config;
  PoseEstimate last_pose;
};

PoseEstimator::PoseEstimator(const CameraIntrinsics& camera,
                             const TargetGeometry& target,
                             const PoseConfig& config)
    : impl_(new Impl()) {
  impl_->camera = camera;
  impl_->target = target;
  impl_->config = config;
}

PoseEstimator::~PoseEstimator() = default;

PoseEstimate PoseEstimator::estimate(const Detection& detection) {
  (void)detection;
  PoseEstimate pose;
  pose.valid = false;
  return pose;
}

void PoseEstimator::reset() { impl_->last_pose = PoseEstimate(); }

void PoseEstimator::projectDisc(const PoseEstimate& pose, int samples,
                                std::vector<cv::Point2f>* points) const {
  (void)pose;
  (void)samples;
  if (points != nullptr) {
    points->clear();
  }
}

}  // namespace glowtrack
