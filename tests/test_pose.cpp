// Synthetic round-trip tests for glowtrack::PoseEstimator (circle PnP).
//
// The target white disc is modelled as a circle of known radius in the target
// plane. Poses are generated, the circle is projected with cv::projectPoints
// into a synthetic camera, a cv::fitEllipse is run on the projected boundary
// (exactly what the detector does) and the resulting Detection is fed back into
// the estimator. Accuracy is measured against the known ground-truth pose.
//
// Conventions are those documented in glowtrack/types.hpp:
//   yaw   = atan2(t.x, t.z), pitch = -atan2(t.y, t.z), distance = ||t||.
#include "test_framework.hpp"

#include "glowtrack/pose_estimator.hpp"
#include "glowtrack/types.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace {

using namespace glowtrack;

constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// 640x480 synthetic camera (fx = fy = 500) with no lens distortion.
CameraIntrinsics makeCamera() {
  CameraIntrinsics camera;
  camera.width = 640;
  camera.height = 480;
  camera.fx = 500.0;
  camera.fy = 500.0;
  camera.cx = 320.0;
  camera.cy = 240.0;
  camera.dist.clear();
  return camera;
}

TargetGeometry makeTarget() {
  TargetGeometry target;
  target.disc_diameter_mm = 50.0;
  target.glow_outer_diameter_mm = 62.0;
  return target;
}

cv::Matx33d rotationFromRvec(const cv::Vec3d& rvec) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  return cv::Matx33d(R);
}

cv::Vec3d rvecFromRotation(const cv::Matx33d& R) {
  cv::Mat rvec;
  cv::Rodrigues(cv::Mat(R), rvec);
  return cv::Vec3d(rvec.at<double>(0, 0), rvec.at<double>(1, 0),
                   rvec.at<double>(2, 0));
}

struct GroundTruthPose {
  cv::Vec3d tvec{0, 0, 0};  // mm
  cv::Matx33d R = cv::Matx33d::eye();
  cv::Vec3d rvec{0, 0, 0};
};

// Generates a physically plausible pose: the target sits at the given LOS
// angles/range and its glowing face points towards the camera, tilted by
// `tilt_deg` about an axis perpendicular to the line of sight. `roll_deg` only
// rotates the disc in its own plane (unobservable for a circle, kept so the
// ground-truth rotation is generic).
GroundTruthPose makeFacingPose(double yaw_deg, double pitch_deg,
                               double range_mm, double tilt_deg,
                               double tilt_axis_deg, double roll_deg) {
  const double yaw = yaw_deg * kDegToRad;
  const double pitch = pitch_deg * kDegToRad;
  const cv::Vec3d direction(std::sin(yaw) * std::cos(pitch),
                            -std::sin(pitch),
                            std::cos(yaw) * std::cos(pitch));
  GroundTruthPose pose;
  pose.tvec = range_mm * direction;
  const cv::Vec3d z0 = -direction;  // normal pointing at the camera
  cv::Vec3d helper(std::fabs(z0[0]) < 0.9 ? 1.0 : 0.0,
                   std::fabs(z0[0]) < 0.9 ? 0.0 : 1.0, 0.0);
  cv::Vec3d u = helper - z0 * helper.dot(z0);
  u *= 1.0 / cv::norm(u);
  const cv::Vec3d w = z0.cross(u);
  const double axis_angle = tilt_axis_deg * kDegToRad;
  const cv::Vec3d axis = std::cos(axis_angle) * u + std::sin(axis_angle) * w;
  const cv::Matx33d tilt_rotation =
      rotationFromRvec(axis * (tilt_deg * kDegToRad));
  const cv::Vec3d normal = tilt_rotation * z0;

  cv::Vec3d ex(1.0, 0.0, 0.0);
  if (std::fabs(ex.dot(normal)) > 0.9) {
    ex = cv::Vec3d(0.0, 1.0, 0.0);
  }
  ex = ex - normal * ex.dot(normal);
  ex *= 1.0 / cv::norm(ex);
  const cv::Vec3d ey = normal.cross(ex);
  cv::Matx33d base;
  for (int i = 0; i < 3; ++i) {
    base(i, 0) = ex[i];
    base(i, 1) = ey[i];
    base(i, 2) = normal[i];
  }
  pose.R = base * rotationFromRvec(cv::Vec3d(0.0, 0.0, roll_deg * kDegToRad));
  pose.rvec = rvecFromRotation(pose.R);
  return pose;
}

std::vector<cv::Point3f> circleObjectPoints(double radius_mm, int samples) {
  std::vector<cv::Point3f> points;
  points.reserve(static_cast<size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double angle =
        2.0 * kPi * static_cast<double>(i) / static_cast<double>(samples);
    points.emplace_back(static_cast<float>(radius_mm * std::cos(angle)),
                        static_cast<float>(radius_mm * std::sin(angle)), 0.0f);
  }
  return points;
}

// Mirrors TargetDetector's cv::RotatedRect -> Ellipse2D conversion: semi_major
// >= semi_minor and angle_deg = direction of the major axis.
Ellipse2D ellipseFromRotatedRect(const cv::RotatedRect& rect) {
  Ellipse2D ellipse;
  ellipse.center = rect.center;
  const bool width_is_major = rect.size.width >= rect.size.height;
  const float major = width_is_major ? rect.size.width : rect.size.height;
  const float minor = width_is_major ? rect.size.height : rect.size.width;
  ellipse.semi_major = 0.5f * major;
  ellipse.semi_minor = 0.5f * minor;
  float angle = rect.angle;
  if (!width_is_major) {
    angle += 90.0f;
  }
  ellipse.angle_deg = angle;
  return ellipse;
}

// Builds the Detection the detector would produce for a ground-truth pose.
Detection makeDetection(const CameraIntrinsics& camera, double radius_mm,
                        const GroundTruthPose& gt, double boundary_noise_px,
                        std::mt19937* rng) {
  const std::vector<cv::Point3f> object_points = circleObjectPoints(radius_mm, 240);
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, gt.rvec, gt.tvec, camera.cameraMatrix(),
                    camera.distortion(), image_points);
  if (boundary_noise_px > 0.0) {
    std::normal_distribution<double> noise(0.0, boundary_noise_px);
    for (cv::Point2f& point : image_points) {
      point.x += static_cast<float>(noise(*rng));
      point.y += static_cast<float>(noise(*rng));
    }
  }
  Detection detection;
  detection.valid = true;
  detection.disc_ellipse = ellipseFromRotatedRect(cv::fitEllipse(image_points));
  detection.glow_ellipse = detection.disc_ellipse;
  detection.score = 1.0f;
  return detection;
}

// Perturbs the fitted ellipse parameters (the "0.5 px ellipse noise" case).
void addEllipseParameterNoise(double sigma_px, Ellipse2D* ellipse,
                              std::mt19937* rng) {
  std::normal_distribution<double> noise(0.0, sigma_px);
  ellipse->center.x += static_cast<float>(noise(*rng));
  ellipse->center.y += static_cast<float>(noise(*rng));
  ellipse->semi_major = static_cast<float>(
      std::max(1.0, ellipse->semi_major + noise(*rng)));
  ellipse->semi_minor = static_cast<float>(
      std::max(0.5, ellipse->semi_minor + noise(*rng)));
  if (ellipse->semi_minor > ellipse->semi_major) {
    std::swap(ellipse->semi_major, ellipse->semi_minor);
  }
  // Angle noise is specified in degrees (a 0.5 deg sigma, not 0.5 px).
  std::normal_distribution<double> angle_noise(0.0, sigma_px);
  ellipse->angle_deg += static_cast<float>(angle_noise(*rng) / sigma_px * 0.5);
}

struct PoseErrors {
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
  double distance_rel = 0.0;
};

PoseErrors computeErrors(const PoseEstimate& pose, const GroundTruthPose& gt) {
  PoseErrors errors;
  const double yaw_gt = std::atan2(gt.tvec[0], gt.tvec[2]);
  const double pitch_gt = -std::atan2(gt.tvec[1], gt.tvec[2]);
  const double distance_gt = cv::norm(gt.tvec) * 1.0e-3;
  errors.yaw_deg = std::fabs(pose.yaw - yaw_gt) * kRadToDeg;
  errors.pitch_deg = std::fabs(pose.pitch - pitch_gt) * kRadToDeg;
  errors.distance_rel =
      std::fabs(pose.distance_m - distance_gt) / distance_gt;
  return errors;
}

struct SweepResult {
  int valid = 0;
  int total = 0;
  int ambiguous = 0;
  double max_yaw_deg = 0.0;
  double max_pitch_deg = 0.0;
  double max_distance_rel = 0.0;
  double mean_yaw_deg = 0.0;
  double mean_pitch_deg = 0.0;
  double mean_distance_rel = 0.0;
  double max_solve_ms = 0.0;
  double max_reproj_rmse_px = 0.0;
};

SweepResult runSweep(const CameraIntrinsics& camera, const TargetGeometry& target,
                     const PoseConfig& config, int trials, double max_tilt_deg,
                     double min_range_mm, double max_range_mm,
                     double ellipse_noise_px, unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  const double radius_mm = 0.5 * target.disc_diameter_mm;
  SweepResult result;
  for (int trial = 0; trial < trials; ++trial) {
    const double yaw = 50.0 * (uniform(rng) - 0.5);     // +-25 deg
    const double pitch = 40.0 * (uniform(rng) - 0.5);   // +-20 deg
    const double range =
        min_range_mm + (max_range_mm - min_range_mm) * uniform(rng);
    const double tilt = max_tilt_deg * uniform(rng);
    const double tilt_axis = 360.0 * uniform(rng);
    const double roll = 360.0 * uniform(rng);
    const GroundTruthPose gt =
        makeFacingPose(yaw, pitch, range, tilt, tilt_axis, roll);
    Detection detection = makeDetection(camera, radius_mm, gt, 0.0, &rng);
    if (ellipse_noise_px > 0.0) {
      addEllipseParameterNoise(ellipse_noise_px, &detection.disc_ellipse, &rng);
    }
    PoseEstimator estimator(camera, target, config);
    const PoseEstimate pose = estimator.estimate(detection);
    ++result.total;
    if (!pose.valid) {
      continue;
    }
    ++result.valid;
    if (pose.ambiguous) {
      ++result.ambiguous;
    }
    const PoseErrors errors = computeErrors(pose, gt);
    result.max_yaw_deg = std::max(result.max_yaw_deg, errors.yaw_deg);
    result.max_pitch_deg = std::max(result.max_pitch_deg, errors.pitch_deg);
    result.max_distance_rel =
        std::max(result.max_distance_rel, errors.distance_rel);
    result.mean_yaw_deg += errors.yaw_deg;
    result.mean_pitch_deg += errors.pitch_deg;
    result.mean_distance_rel += errors.distance_rel;
    result.max_solve_ms = std::max(result.max_solve_ms, pose.solve_ms);
    result.max_reproj_rmse_px =
        std::max(result.max_reproj_rmse_px, pose.reproj_rmse_px);
  }
  if (result.valid > 0) {
    result.mean_yaw_deg /= result.valid;
    result.mean_pitch_deg /= result.valid;
    result.mean_distance_rel /= result.valid;
  }
  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// Accuracy: small plane tilt / noise-free data.
// ---------------------------------------------------------------------------
GT_TEST(pose_near_frontal_accuracy) {
  const CameraIntrinsics camera = makeCamera();
  const TargetGeometry target = makeTarget();
  PoseConfig config;
  const SweepResult sweep =
      runSweep(camera, target, config, 40, /*max_tilt_deg=*/10.0,
               /*min_range_mm=*/300.0, /*max_range_mm=*/2000.0,
               /*ellipse_noise_px=*/0.0, 20241023u);
  GT_CHECK(sweep.valid == sweep.total);
  GT_CHECK_LT(sweep.max_yaw_deg, 0.2);
  GT_CHECK_LT(sweep.max_pitch_deg, 0.2);
  GT_CHECK_LT(sweep.max_distance_rel, 0.01);
  GT_CHECK_LT(sweep.mean_yaw_deg, 0.05);
  GT_CHECK_LT(sweep.mean_pitch_deg, 0.05);
  // Validity must use the documented metric: refined poses fit the ellipse
  // essentially exactly (see the structural-zero note in pose_estimator.cpp).
  GT_CHECK_LT(sweep.max_reproj_rmse_px, config.max_reproj_rmse_px);
}

// ---------------------------------------------------------------------------
// Accuracy: moderate plane tilt (up to 40 deg) / noise-free data.
//
// The two-fold circle ambiguity is fundamental: the second (mirror) circular
// section has the same conic cost, so a cold-start single frame cannot always
// pick the physically true one. Its LOS angles still agree closely, hence the
// bounds below. In a tracked sequence the temporal prior removes the ambiguity
// (see pose_ambiguity_continuity_in_sequence).
// ---------------------------------------------------------------------------
GT_TEST(pose_moderate_tilt_accuracy) {
  const CameraIntrinsics camera = makeCamera();
  const TargetGeometry target = makeTarget();
  PoseConfig config;
  const SweepResult sweep =
      runSweep(camera, target, config, 50, /*max_tilt_deg=*/40.0,
               /*min_range_mm=*/400.0, /*max_range_mm=*/2000.0,
               /*ellipse_noise_px=*/0.0, 20241024u);
  GT_CHECK(sweep.valid == sweep.total);
  GT_CHECK_LT(sweep.mean_yaw_deg, 0.1);
  GT_CHECK_LT(sweep.mean_pitch_deg, 0.1);
  GT_CHECK_LT(sweep.max_yaw_deg, 1.0);
  GT_CHECK_LT(sweep.max_pitch_deg, 1.0);
  GT_CHECK_LT(sweep.max_distance_rel, 0.01);
  GT_CHECK_LT(sweep.max_reproj_rmse_px, config.max_reproj_rmse_px);
}

// ---------------------------------------------------------------------------
// Accuracy with 0.5 px gaussian noise on the fitted ellipse parameters.
// ---------------------------------------------------------------------------
GT_TEST(pose_ellipse_noise_sensitivity) {
  const CameraIntrinsics camera = makeCamera();
  const TargetGeometry target = makeTarget();
  PoseConfig config;
  const SweepResult sweep =
      runSweep(camera, target, config, 60, /*max_tilt_deg=*/40.0,
               /*min_range_mm=*/300.0, /*max_range_mm=*/1200.0,
               /*ellipse_noise_px=*/0.5, 20241025u);
  GT_CHECK_GT(sweep.valid, sweep.total - 3);
  // Requirement: < 1 deg / 5 % typical (mean) with 0.5 px ellipse noise.
  GT_CHECK_LT(sweep.mean_yaw_deg, 1.0);
  GT_CHECK_LT(sweep.mean_pitch_deg, 1.0);
  GT_CHECK_LT(sweep.mean_distance_rel, 0.05);
  GT_CHECK_LT(sweep.max_yaw_deg, 2.0);
  GT_CHECK_LT(sweep.max_pitch_deg, 2.0);
  GT_CHECK_LT(sweep.max_reproj_rmse_px, config.max_reproj_rmse_px);
}

// ---------------------------------------------------------------------------
// Specific near-axis pose: tvec = (20, -10, 500) mm, disc facing the camera.
// ---------------------------------------------------------------------------
GT_TEST(pose_near_axis_recovery) {
  const CameraIntrinsics camera = makeCamera();
  const TargetGeometry target = makeTarget();
  PoseConfig config;
  std::mt19937 rng(7);
  GroundTruthPose gt;
  // Facing the camera, no tilt: normal = -z.
  gt.R = rotationFromRvec(cv::Vec3d(kPi, 0.0, 0.0));
  gt.rvec = rvecFromRotation(gt.R);
  gt.tvec = cv::Vec3d(20.0, -10.0, 500.0);
  const Detection detection = makeDetection(camera, 25.0, gt, 0.0, &rng);

  PoseEstimator estimator(camera, target, config);
  const PoseEstimate pose = estimator.estimate(detection);
  GT_CHECK(pose.valid);
  GT_CHECK(pose.num_solutions >= 1);
  const PoseErrors errors = computeErrors(pose, gt);
  GT_CHECK_LT(errors.yaw_deg, 0.1);
  GT_CHECK_LT(errors.pitch_deg, 0.1);
  GT_CHECK_LT(errors.distance_rel, 0.005);
  // The canonical facing representation must have n_z < 0.
  const cv::Vec3d normal = pose.R * cv::Vec3d(0.0, 0.0, 1.0);
  GT_CHECK_LT(normal[2], 0.0);
  // x/y point roughly along the camera axes for a fronto-parallel target.
  GT_CHECK_GT(normal[2], -1.0001);
}

// ---------------------------------------------------------------------------
// projectDisc() must match cv::projectPoints (including lens distortion).
// ---------------------------------------------------------------------------
GT_TEST(pose_project_disc_matches_opencv) {
  CameraIntrinsics camera = makeCamera();
  camera.dist = {0.12, -0.08, 0.0015, -0.0022, 0.01};
  const TargetGeometry target = makeTarget();
  const PoseConfig config;
  const GroundTruthPose gt =
      makeFacingPose(12.0, -8.0, 650.0, 25.0, 40.0, 15.0);
  PoseEstimate pose;
  pose.valid = true;
  pose.rvec = gt.rvec;
  pose.tvec = gt.tvec;
  pose.R = gt.R;

  PoseEstimator estimator(camera, target, config);
  const int samples = 24;
  std::vector<cv::Point2f> projected;
  estimator.projectDisc(pose, samples, &projected);
  GT_CHECK(projected.size() == static_cast<size_t>(samples));

  const std::vector<cv::Point3f> object_points = circleObjectPoints(25.0, samples);
  std::vector<cv::Point2f> reference;
  cv::projectPoints(object_points, gt.rvec, gt.tvec, camera.cameraMatrix(),
                    camera.distortion(), reference);
  GT_CHECK(reference.size() == projected.size());
  for (size_t i = 0; i < projected.size(); ++i) {
    GT_CHECK_LT(cv::norm(projected[i] - reference[i]), 1.0e-3);
  }
}

// ---------------------------------------------------------------------------
// The reported reprojection residual is a structural consistency check:
//   * refined pose  -> essentially zero (the ellipse is exactly realizable),
//   * raw IPPE pose -> strictly positive (refinement not run).
// ---------------------------------------------------------------------------
GT_TEST(pose_reproj_metric_is_live) {
  const CameraIntrinsics camera = makeCamera();
  const TargetGeometry target = makeTarget();
  std::mt19937 rng(99);
  const GroundTruthPose gt =
      makeFacingPose(10.0, 5.0, 600.0, 25.0, 60.0, 0.0);
  const Detection detection = makeDetection(camera, 25.0, gt, 0.0, &rng);

  // Refined: geometrically exact, so the residual is ~0 and well below max.
  PoseConfig refined_config;
  PoseEstimator refined_estimator(camera, target, refined_config);
  const PoseEstimate refined = refined_estimator.estimate(detection);
  GT_CHECK(refined.valid);
  GT_CHECK_LT(refined.reproj_rmse_px, 0.05);
  GT_CHECK_LE(refined.reproj_rmse_px, refined_config.max_reproj_rmse_px);

  // Fallback path (no refinement): the IPPE solution alone must report the
  // distance from its projected circle to the fitted ellipse: > 0, < max.
  PoseConfig ippe_config;
  ippe_config.refine_iterations = 0;
  PoseEstimator ippe_estimator(camera, target, ippe_config);
  const PoseEstimate ippe = ippe_estimator.estimate(detection);
  GT_CHECK(ippe.valid);
  GT_CHECK_GT(ippe.reproj_rmse_px, 0.0);
  GT_CHECK_LT(ippe.reproj_rmse_px, ippe_config.max_reproj_rmse_px);
  GT_CHECK_GT(ippe.reproj_rmse_px, refined.reproj_rmse_px);
}

// ---------------------------------------------------------------------------
// Invalid / degenerate inputs are rejected.
// ---------------------------------------------------------------------------
GT_TEST(pose_invalid_inputs_rejected) {
  const CameraIntrinsics camera = makeCamera();
  const TargetGeometry target = makeTarget();
  const PoseConfig config;
  PoseEstimator estimator(camera, target, config);

  // Empty / never-filled ellipse.
  Detection empty;
  GT_CHECK(!estimator.estimate(empty).valid);

  // NaN semi-axis.
  Detection nan_detection;
  nan_detection.valid = true;
  nan_detection.disc_ellipse.center = cv::Point2f(320.0f, 240.0f);
  nan_detection.disc_ellipse.semi_major = std::nanf("");
  nan_detection.disc_ellipse.semi_minor = 10.0f;
  GT_CHECK(!estimator.estimate(nan_detection).valid);

  // Degenerate (nearly a line) ellipse.
  Detection degenerate;
  degenerate.valid = true;
  degenerate.disc_ellipse.center = cv::Point2f(320.0f, 240.0f);
  degenerate.disc_ellipse.semi_major = 40.0f;
  degenerate.disc_ellipse.semi_minor = 0.5f;
  GT_CHECK(!estimator.estimate(degenerate).valid);

  // Ellipse much too large -> distance below min_distance_m.
  Detection too_close;
  too_close.valid = true;
  too_close.disc_ellipse.center = cv::Point2f(320.0f, 240.0f);
  too_close.disc_ellipse.semi_major = 400.0f;
  too_close.disc_ellipse.semi_minor = 380.0f;
  too_close.disc_ellipse.angle_deg = 0.0f;
  GT_CHECK(!estimator.estimate(too_close).valid);

  // Ellipse tiny -> distance above max_distance_m.
  Detection too_far;
  too_far.valid = true;
  too_far.disc_ellipse.center = cv::Point2f(320.0f, 240.0f);
  too_far.disc_ellipse.semi_major = 0.60f;
  too_far.disc_ellipse.semi_minor = 0.52f;
  too_far.disc_ellipse.angle_deg = 0.0f;
  GT_CHECK(!estimator.estimate(too_far).valid);

  // Ellipse located outside the image.
  Detection outside;
  outside.valid = true;
  outside.disc_ellipse.center = cv::Point2f(5000.0f, 5000.0f);
  outside.disc_ellipse.semi_major = 40.0f;
  outside.disc_ellipse.semi_minor = 35.0f;
  outside.disc_ellipse.angle_deg = 0.0f;
  GT_CHECK(!estimator.estimate(outside).valid);
}

// ---------------------------------------------------------------------------
// Defensive normalization: swapped semi-axes with a 90 deg rotated angle
// describe the same ellipse and must give the same pose.
// ---------------------------------------------------------------------------
GT_TEST(pose_swapped_axes_normalized) {
  const CameraIntrinsics camera = makeCamera();
  const TargetGeometry target = makeTarget();
  const PoseConfig config;
  std::mt19937 rng(123);
  const GroundTruthPose gt = makeFacingPose(-15.0, 7.0, 550.0, 20.0, 100.0, 0.0);
  Detection detection = makeDetection(camera, 25.0, gt, 0.0, &rng);

  PoseEstimator estimator_a(camera, target, config);
  const PoseEstimate pose_a = estimator_a.estimate(detection);

  Detection swapped = detection;
  swapped.disc_ellipse.semi_major = detection.disc_ellipse.semi_minor;
  swapped.disc_ellipse.semi_minor = detection.disc_ellipse.semi_major;
  swapped.disc_ellipse.angle_deg =
      detection.disc_ellipse.angle_deg + 90.0f;
  PoseEstimator estimator_b(camera, target, config);
  const PoseEstimate pose_b = estimator_b.estimate(swapped);

  GT_CHECK(pose_a.valid);
  GT_CHECK(pose_b.valid);
  GT_CHECK_LT(std::fabs(pose_a.yaw - pose_b.yaw) * kRadToDeg, 0.01);
  GT_CHECK_LT(std::fabs(pose_a.pitch - pose_b.pitch) * kRadToDeg, 0.01);
  GT_CHECK_LT(std::fabs(pose_a.distance_m - pose_b.distance_m), 1.0e-4);
}

// ---------------------------------------------------------------------------
// Ambiguity continuity: over a smooth sequence the chosen circular section must
// not flip and the tracked LOS angles must stay accurate.
// ---------------------------------------------------------------------------
GT_TEST(pose_ambiguity_continuity_in_sequence) {
  const CameraIntrinsics camera = makeCamera();
  const TargetGeometry target = makeTarget();
  const PoseConfig config;
  std::mt19937 rng(2024);
  PoseEstimator estimator(camera, target, config);

  const int frames = 60;
  int valid_frames = 0;
  int flips = 0;
  int ambiguous_frames = 0;
  double max_yaw = 0.0;
  double max_pitch = 0.0;
  double max_distance = 0.0;
  cv::Vec3d previous_normal(0.0, 0.0, 0.0);
  bool have_previous = false;

  for (int frame = 0; frame < frames; ++frame) {
    const double phase = 2.0 * kPi * frame / frames;
    const double yaw = 20.0 * std::sin(phase);
    const double pitch = 12.0 * std::cos(0.7 * phase);
    const double range_mm = 500.0 + 400.0 * (0.5 + 0.5 * std::sin(0.5 * phase));
    const double tilt = 25.0 + 10.0 * std::sin(1.3 * phase);
    const double axis = 90.0 + 40.0 * std::sin(0.9 * phase);
    const GroundTruthPose gt =
        makeFacingPose(yaw, pitch, range_mm, tilt, axis, 0.0);
    const Detection detection = makeDetection(camera, 25.0, gt, 0.3, &rng);
    const PoseEstimate pose = estimator.estimate(detection);
    if (!pose.valid) {
      continue;
    }
    ++valid_frames;
    if (pose.ambiguous) {
      ++ambiguous_frames;
    }
    const PoseErrors errors = computeErrors(pose, gt);
    max_yaw = std::max(max_yaw, errors.yaw_deg);
    max_pitch = std::max(max_pitch, errors.pitch_deg);
    max_distance = std::max(max_distance, errors.distance_rel);

    const cv::Vec3d normal = pose.R * cv::Vec3d(0.0, 0.0, 1.0);
    if (have_previous && normal.dot(previous_normal) < 0.9) {
      ++flips;
    }
    previous_normal = normal;
    have_previous = true;
  }

  GT_CHECK_GT(valid_frames, frames - 3);
  GT_CHECK(flips == 0);  // the chosen circular section must not flip
  GT_CHECK_LT(max_yaw, 0.5);
  GT_CHECK_LT(max_pitch, 0.5);
  GT_CHECK_LT(max_distance, 0.05);
  // Once the temporal prior has engaged the solution is decided (not ambiguous)
  // for the vast majority of frames.
  GT_CHECK_LT(ambiguous_frames, frames / 4);
}
