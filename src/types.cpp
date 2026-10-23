#include "glowtrack/types.hpp"

#include <algorithm>
#include <cmath>

namespace glowtrack {

cv::Mat CameraIntrinsics::cameraMatrix() const {
  return (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
}

cv::Mat CameraIntrinsics::distortion() const {
  if (dist.empty()) {
    return cv::Mat::zeros(1, 5, CV_64F);
  }
  cv::Mat d(1, static_cast<int>(dist.size()), CV_64F);
  for (int i = 0; i < static_cast<int>(dist.size()); ++i) {
    d.at<double>(0, i) = dist[static_cast<size_t>(i)];
  }
  return d;
}

float Ellipse2D::area() const {
  return static_cast<float>(kPi) * semi_major * semi_minor;
}

float Ellipse2D::mean_radius() const {
  return std::sqrt(std::max(0.f, semi_major * semi_minor));
}

bool Ellipse2D::valid() const {
  return std::isfinite(center.x) && std::isfinite(center.y) &&
         std::isfinite(semi_major) && std::isfinite(semi_minor) &&
         semi_major > 0.f && semi_minor > 0.f;
}

const char* toString(TrackState state) {
  switch (state) {
    case TrackState::Idle:
      return "IDLE";
    case TrackState::Searching:
      return "SEARCH";
    case TrackState::Acquiring:
      return "ACQUIRE";
    case TrackState::Tracking:
      return "TRACK";
    case TrackState::Predicting:
      return "PREDICT";
    case TrackState::Lost:
      return "LOST";
  }
  return "UNKNOWN";
}

}  // namespace glowtrack
