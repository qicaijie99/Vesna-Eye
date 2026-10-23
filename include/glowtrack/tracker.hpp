#pragma once
// Target tracker: orchestrates detector -> PnP -> Kalman filters + state machine.
//
// update() is called once per camera frame and returns the line-of-sight angles
// that are sent to the lower computer. The tracker keeps the search ROI, the
// track state (searching / acquiring / tracking / predicting), resolves pose
// ambiguity, rejects outliers and extrapolates angles through detection gaps.

#include "glowtrack/config.hpp"
#include "glowtrack/types.hpp"

#include <memory>
#include <string>

namespace glowtrack {

class TargetTracker {
 public:
  explicit TargetTracker(const RuntimeConfig& config);
  ~TargetTracker();

  TargetTracker(const TargetTracker&) = delete;
  TargetTracker& operator=(const TargetTracker&) = delete;

  // Processes one frame. `timestamp_s` should be a monotonically increasing
  // capture time; `frame_index` is only used for logging.
  TrackOutput update(const cv::Mat& bgr, double timestamp_s, int frame_index);

  void reset();

  // Debug / overlay accessors (valid after update()).
  const Detection& lastDetection() const;
  const PoseEstimate& lastPose() const;
  cv::Rect currentSearchRoi() const;
  TrackState state() const;
  std::string stateName() const;
  double lastProcessingMs() const;

  const RuntimeConfig& config() const;

 private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace glowtrack
