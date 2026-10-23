#pragma once
// PnP pose estimation from the detected white disc ellipse.
//
// The target's white disc is a circle of known physical radius R. Its image is
// an ellipse, which fully determines the pose of the circle up to the classical
// two-fold ambiguity of a planar circle:
//
//   1. build four initial 2D-3D correspondences from the ellipse axes
//      (affine-consistent initialization),
//   2. solve with cv::solvePnPGeneric(..., SOLVEPNP_IPPE) -> up to 2 poses,
//   3. pick the physically meaningful solution (target face towards camera,
//      temporal continuity with the previous accepted pose),
//   4. Gauss-Newton refine the pose so that the projected model circle matches
//      the observed ellipse (no correspondence bias),
//   5. output LOS angles yaw/pitch and distance from tvec, plus fit quality.

#include "glowtrack/config.hpp"
#include "glowtrack/types.hpp"

#include <memory>
#include <vector>

namespace glowtrack {

class PoseEstimator {
 public:
  PoseEstimator(const CameraIntrinsics& camera, const TargetGeometry& target,
                const PoseConfig& config);
  ~PoseEstimator();

  PoseEstimator(const PoseEstimator&) = delete;
  PoseEstimator& operator=(const PoseEstimator&) = delete;

  // Estimates the target pose for one detection. Ambiguity resolution uses the
  // previously accepted pose as a continuity prior when available.
  //
  // Reported quality metrics:
  //   PoseEstimate::reproj_rmse_px is the RMS first-order geometric distance
  //   between the 48 projected model-circle points of the final pose and the
  //   fitted disc ellipse. NOTE: for a circle of known radius every ellipse is
  //   exactly realizable by a circle pose (up to the two-fold ambiguity), so a
  //   converged refinement reports ~0; a clearly positive value means the
  //   refinement failed and the raw IPPE solution is being used (still useful
  //   as a failure detector, not as a detector-noise metric).
  PoseEstimate estimate(const Detection& detection);

  // Forget the temporal prior (call between independent sequences).
  void reset();

  // Projected circle of a given pose with the configured camera model
  // (used by tests/debug: returns N points around the disc).
  void projectDisc(const PoseEstimate& pose, int samples,
                   std::vector<cv::Point2f>* points) const;

 private:
  struct Impl;

  std::unique_ptr<Impl> impl_;
};

}  // namespace glowtrack
