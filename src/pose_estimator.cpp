// Pose estimation of the glowing target's white disc from its image ellipse.
//
// The white disc is a circle of known physical radius R that lives in the
// target plane z = 0 (target frame). Its perspective image is an ellipse, which
// determines the circle pose up to the classical two-fold ambiguity of a planar
// circle. The implementation follows the classical pipeline:
//
//   1. affine-consistent initialization: the ellipse's principal axes define
//      four 2D-3D correspondences (circle points at 0/90/180/270 deg),
//   2. cv::solvePnPGeneric(..., SOLVEPNP_IPPE) returns the (up to two) pose
//      candidates,
//   3. every candidate is canonicalized so the target face normal points
//      towards the camera (n_z < 0) -- for a circle the opposite normal
//      describes the same physical disc,
//   4. each candidate is refined by a Levenberg-Marquardt loop that minimizes
//      the projected model circle against the observed ellipse conic. The
//      residual is the first-order geometric distance F(p)/|grad F(p)| of the
//      projected circle samples, so no 2D-3D correspondence bias is baked in,
//   5. the candidates are compared (agreement / facing sanity / reprojection
//      cost / temporal continuity) and the best one is returned,
//   6. the pose is validated against the configured distance range and the
//      maximum reprojection RMSE.
//
// The reprojection RMSE that is reported is the RMS of |F(p)|/|grad F(p)| over
// the projected circle samples, i.e. a geometric distance in pixels.

#include "glowtrack/pose_estimator.hpp"

#include "glowtrack/util.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace glowtrack {
namespace {

// Optional diagnostic logging (set GT_POSE_DEBUG=1 in the environment).
bool debugEnabled() {
  static const bool enabled = []() {
    const char* value = std::getenv("GT_POSE_DEBUG");
    return value != nullptr && value[0] != '\0' && value[0] != '0';
  }();
  return enabled;
}

#define GT_POSE_LOG(...)                            \
  do {                                              \
    if (glowtrack::debugEnabled()) {                \
      std::fprintf(stderr, __VA_ARGS__);            \
    }                                               \
  } while (0)

// Number of circle samples used by the nonlinear refinement and by the
// reprojection cost. 48 samples are dense enough for a stable conic cost and
// cheap enough for real-time use.
constexpr int kRefineSamples = 48;

constexpr double kInvalidCost = 1.0e6;

// Candidate faces are considered "grazing" when the normal is almost parallel
// to the image plane; such a circle is nearly a line and its pose is unstable.
constexpr double kMinFacingNz = 0.15;

// Two refined candidates closer than this are the same pose (the two-fold
// ambiguity is numerically degenerate for near-frontal targets).
constexpr double kSamePoseNormalDot = 0.9995;
constexpr double kSamePoseTranslationRatio = 1.0e-3;  // relative to |t|
constexpr double kSamePoseTranslationAbs = 0.5;      // mm

// ---------------------------------------------------------------------------
// Camera model (pinhole + OpenCV radial/tangential distortion)
// ---------------------------------------------------------------------------
struct CameraModel {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  double k3 = 0.0;
  bool has_distortion = false;

  static CameraModel from(const CameraIntrinsics& camera) {
    CameraModel model;
    model.fx = camera.fx;
    model.fy = camera.fy;
    model.cx = camera.cx;
    model.cy = camera.cy;
    if (camera.dist.size() >= 2) {
      // OpenCV order: k1, k2, p1, p2, k3.
      model.k1 = camera.dist[0];
      model.k2 = camera.dist[1];
      model.has_distortion = true;
    }
    if (camera.dist.size() >= 4) {
      model.p1 = camera.dist[2];
      model.p2 = camera.dist[3];
    }
    if (camera.dist.size() >= 5) {
      model.k3 = camera.dist[4];
    }
    return model;
  }

  bool valid() const {
    return std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) &&
           std::isfinite(cy) && fx > 0.0 && fy > 0.0;
  }

  // Projects a target-frame point through a pose (X_cam = R * P + t).
  // Returns false when the point is behind the camera or not finite.
  bool project(const cv::Matx33d& R, const cv::Vec3d& t, const cv::Vec3d& P,
               cv::Point2d* image) const {
    const double X = R(0, 0) * P[0] + R(0, 1) * P[1] + R(0, 2) * P[2] + t[0];
    const double Y = R(1, 0) * P[0] + R(1, 1) * P[1] + R(1, 2) * P[2] + t[1];
    const double Z = R(2, 0) * P[0] + R(2, 1) * P[1] + R(2, 2) * P[2] + t[2];
    if (!std::isfinite(X) || !std::isfinite(Y) || !std::isfinite(Z) ||
        Z <= 1.0e-6) {
      return false;
    }
    double x = X / Z;
    double y = Y / Z;
    if (has_distortion) {
      const double r2 = x * x + y * y;
      const double radial =
          1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
      const double xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
      const double yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
      x = xd;
      y = yd;
    }
    image->x = fx * x + cx;
    image->y = fy * y + cy;
    return std::isfinite(image->x) && std::isfinite(image->y);
  }
};

// ---------------------------------------------------------------------------
// Observed ellipse as an implicit conic  F(p) = d^T Q d - 1,  d = p - center
// ---------------------------------------------------------------------------
struct EllipseConic {
  cv::Point2d center{0.0, 0.0};
  double a = 0.0;      // semi-major axis [px]
  double b = 0.0;      // semi-minor axis [px]
  double theta = 0.0;  // orientation of the major axis [rad]
  double q00 = 0.0;
  double q01 = 0.0;
  double q11 = 0.0;

  bool valid() const {
    return std::isfinite(center.x) && std::isfinite(center.y) &&
           std::isfinite(a) && std::isfinite(b) && std::isfinite(theta) &&
           a > 0.5 && b > 0.5 && b / a >= 0.05;
  }

  // First-order geometric distance of `point` to the conic (px). It is exact
  // for points on the ellipse and a good approximation nearby, which is the
  // regime the refinement works in.
  double normalizedDistance(const cv::Point2d& point) const {
    const double dx = point.x - center.x;
    const double dy = point.y - center.y;
    const double f = q00 * dx * dx + 2.0 * q01 * dx * dy + q11 * dy * dy - 1.0;
    const double gx = 2.0 * (q00 * dx + q01 * dy);
    const double gy = 2.0 * (q01 * dx + q11 * dy);
    const double g = std::sqrt(gx * gx + gy * gy);
    if (!(g > 1.0e-12)) {
      return 0.0;
    }
    return f / g;
  }
};

// Builds the conic of an image ellipse. The Ellipse2D contract says semi_major
// >= semi_minor and angle_deg is the orientation of the major axis; a defensive
// normalization handles swapped axes as well.
bool buildEllipseConic(const Ellipse2D& ellipse, EllipseConic* conic) {
  if (!ellipse.valid()) {
    return false;
  }
  double a = std::max(static_cast<double>(ellipse.semi_major),
                      static_cast<double>(ellipse.semi_minor));
  double b = std::min(static_cast<double>(ellipse.semi_major),
                      static_cast<double>(ellipse.semi_minor));
  double theta = static_cast<double>(ellipse.angle_deg) * kDeg2Rad;
  if (ellipse.semi_major < ellipse.semi_minor) {
    // The caller passed the minor axis angle: rotate by 90 degrees.
    theta += 0.5 * kPi;
  }
  if (!(a > 0.5) || !(b > 0.5) || !std::isfinite(a) || !std::isfinite(b) ||
      !std::isfinite(theta) || b / a < 0.05) {
    return false;
  }
  conic->center = cv::Point2d(ellipse.center.x, ellipse.center.y);
  conic->a = a;
  conic->b = b;
  conic->theta = theta;
  const double cs = std::cos(theta);
  const double sn = std::sin(theta);
  const double inv_a2 = 1.0 / (a * a);
  const double inv_b2 = 1.0 / (b * b);
  conic->q00 = cs * cs * inv_a2 + sn * sn * inv_b2;
  conic->q01 = cs * sn * (inv_a2 - inv_b2);
  conic->q11 = sn * sn * inv_a2 + cs * cs * inv_b2;
  return true;
}

// ---------------------------------------------------------------------------
// Rotation helpers
// ---------------------------------------------------------------------------
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

// A circle is invariant under a 180 degree rotation about any of its diameters:
// for a circle pose the two representations R and R * Rx(pi) describe exactly
// the same physical disc (the target x axis is unchanged, y and z flip). We
// always use the representation whose target z axis points towards the camera,
// so the target "face normal" is well defined for overlays and debugging.
cv::Matx33d canonicalFacingRotation(const cv::Matx33d& R) {
  if (R(2, 2) > 0.0) {
    static const cv::Matx33d kFlip(1, 0, 0, 0, -1, 0, 0, 0, -1);
    return R * kFlip;
  }
  return R;
}

// ---------------------------------------------------------------------------
// Model circle samples
// ---------------------------------------------------------------------------
std::vector<cv::Vec3d> makeCircleSamples(double radius_mm, int samples) {
  std::vector<cv::Vec3d> points;
  points.reserve(static_cast<size_t>(std::max(4, samples)));
  for (int i = 0; i < samples; ++i) {
    const double angle = 2.0 * kPi * static_cast<double>(i) /
                         static_cast<double>(samples);
    points.emplace_back(radius_mm * std::cos(angle),
                        radius_mm * std::sin(angle), 0.0);
  }
  return points;
}

// Residual vector: normalized conic distance of every projected model point.
// Returns the mean squared residual (cost); the residuals are stored in *out
// when requested.
double evaluateResiduals(const CameraModel& camera, const EllipseConic& conic,
                         const std::vector<cv::Vec3d>& circle_points,
                         const cv::Matx33d& R, const cv::Vec3d& t,
                         std::vector<double>* out) {
  const size_t count = circle_points.size();
  if (out != nullptr) {
    out->resize(count);
  }
  double sum = 0.0;
  for (size_t i = 0; i < count; ++i) {
    cv::Point2d projected;
    double residual;
    if (camera.project(R, t, circle_points[i], &projected)) {
      residual = conic.normalizedDistance(projected);
    } else {
      residual = 1.0e3;  // behind the camera: heavily penalized
    }
    if (!std::isfinite(residual)) {
      residual = 1.0e3;
    }
    if (out != nullptr) {
      (*out)[i] = residual;
    }
    sum += residual * residual;
  }
  const double cost = sum / static_cast<double>(count);
  return std::isfinite(cost) ? cost : kInvalidCost;
}

// RMS first-order geometric distance [px] of the projected model-circle samples
// to the observed ellipse conic. This is the number reported in
// PoseEstimate::reproj_rmse_px.
//
// NOTE: for a *circle* of known radius the reported value is structurally close
// to zero for any fitted ellipse: every ellipse is exactly the projection of two
// circles of the given radius (the two circular sections of the back-projection
// cone). A value well above zero therefore indicates a failed refinement (or an
// intentionally unrefined IPPE fallback), not detector noise.
double computeReprojRmse(const CameraModel& camera, const EllipseConic& conic,
                         const std::vector<cv::Vec3d>& circle_points,
                         const cv::Matx33d& R, const cv::Vec3d& t) {
  const double cost =
      evaluateResiduals(camera, conic, circle_points, R, t, nullptr);
  return std::sqrt(std::max(0.0, cost));
}

// ---------------------------------------------------------------------------
// Levenberg-Marquardt refinement of a 6-DoF pose against the ellipse conic.
// Returns false when the refinement diverges (the caller then keeps the IPPE
// solution).
// ---------------------------------------------------------------------------
bool refinePose(const CameraModel& camera, const EllipseConic& conic,
                const std::vector<cv::Vec3d>& circle_points,
                int max_iterations, const cv::Vec3d& rvec_initial,
                const cv::Vec3d& tvec_initial, cv::Vec3d* rvec_out,
                cv::Vec3d* tvec_out, double* cost_out) {
  double x[6] = {rvec_initial[0], rvec_initial[1], rvec_initial[2],
                 tvec_initial[0], tvec_initial[1], tvec_initial[2]};
  auto evaluate = [&](const double* state) {
    const cv::Matx33d R =
        rotationFromRvec(cv::Vec3d(state[0], state[1], state[2]));
    const cv::Vec3d t(state[3], state[4], state[5]);
    return evaluateResiduals(camera, conic, circle_points, R, t, nullptr);
  };

  double cost = evaluate(x);
  if (!std::isfinite(cost)) {
    return false;
  }

  const double start_cost = cost;
  double lambda = 1.0e-3;
  const double translation_scale = std::max(1.0, cv::norm(tvec_initial));
  bool converged = false;

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    // Numeric Jacobian (central differences). The residual is smooth around
    // the optimum so this is accurate and keeps the code simple.
    std::vector<double> residuals;
    const cv::Matx33d R =
        rotationFromRvec(cv::Vec3d(x[0], x[1], x[2]));
    const cv::Vec3d t(x[3], x[4], x[5]);
    evaluateResiduals(camera, conic, circle_points, R, t, &residuals);
    const size_t count = residuals.size();

    double J[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double H[6][6] = {};
    // residuals[k][i] for the six parameter directions
    std::vector<std::vector<double>> columns(6);
    double steps[6];
    for (int k = 0; k < 3; ++k) {
      steps[k] = 1.0e-5;  // rad
    }
    for (int k = 3; k < 6; ++k) {
      steps[k] = 1.0e-5 * translation_scale;  // mm
    }
    for (int k = 0; k < 6; ++k) {
      double plus[6];
      double minus[6];
      for (int j = 0; j < 6; ++j) {
        plus[j] = x[j];
        minus[j] = x[j];
      }
      plus[k] += steps[k];
      minus[k] -= steps[k];
      std::vector<double> residual_plus;
      std::vector<double> residual_minus;
      evaluateResiduals(
          camera, conic, circle_points,
          rotationFromRvec(cv::Vec3d(plus[0], plus[1], plus[2])),
          cv::Vec3d(plus[3], plus[4], plus[5]), &residual_plus);
      evaluateResiduals(
          camera, conic, circle_points,
          rotationFromRvec(cv::Vec3d(minus[0], minus[1], minus[2])),
          cv::Vec3d(minus[3], minus[4], minus[5]), &residual_minus);
      std::vector<double>& column = columns[static_cast<size_t>(k)];
      column.resize(count);
      for (size_t i = 0; i < count; ++i) {
        column[i] = (residual_plus[i] - residual_minus[i]) / (2.0 * steps[k]);
        J[k] += column[i] * residuals[i];
      }
    }
    for (size_t i = 0; i < count; ++i) {
      for (int a = 0; a < 6; ++a) {
        for (int b = 0; b < 6; ++b) {
          H[a][b] += columns[static_cast<size_t>(a)][i] *
                     columns[static_cast<size_t>(b)][i];
        }
      }
    }

    const double gradient_norm =
        std::sqrt(J[0] * J[0] + J[1] * J[1] + J[2] * J[2] + J[3] * J[3] +
                  J[4] * J[4] + J[5] * J[5]);
    if (gradient_norm < 1.0e-14) {
      converged = true;
      break;
    }

    bool step_accepted = false;
    double next_x[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double next_cost = cost;
    for (int attempt = 0; attempt < 8 && !step_accepted; ++attempt) {
      double A[6][6];
      double b[6];
      for (int a = 0; a < 6; ++a) {
        b[a] = -J[a];
        for (int c = 0; c < 6; ++c) {
          A[a][c] = H[a][c];
          if (a == c) {
            A[a][c] += lambda * (H[a][c] + 1.0e-9);
          }
        }
      }
      // Solve A * delta = b with partial pivoting.
      bool solvable = true;
      for (int col = 0; col < 6 && solvable; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 6; ++row) {
          if (std::fabs(A[row][col]) > std::fabs(A[pivot][col])) {
            pivot = row;
          }
        }
        if (std::fabs(A[pivot][col]) < 1.0e-18) {
          solvable = false;
          break;
        }
        if (pivot != col) {
          for (int c = 0; c < 6; ++c) {
            std::swap(A[col][c], A[pivot][c]);
          }
          std::swap(b[col], b[pivot]);
        }
        const double inverse = 1.0 / A[col][col];
        for (int row = col + 1; row < 6; ++row) {
          const double factor = A[row][col] * inverse;
          if (factor == 0.0) {
            continue;
          }
          for (int c = col; c < 6; ++c) {
            A[row][c] -= factor * A[col][c];
          }
          b[row] -= factor * b[col];
        }
      }
      double delta[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      if (solvable) {
        for (int row = 5; row >= 0; --row) {
          double value = b[row];
          for (int c = row + 1; c < 6; ++c) {
            value -= A[row][c] * delta[c];
          }
          delta[row] = value / A[row][row];
        }
      }
      for (int k = 0; k < 6; ++k) {
        next_x[k] = x[k] + delta[k];
      }
      next_cost = evaluate(next_x);
      if (next_cost < cost) {
        step_accepted = true;
      } else {
        lambda *= 10.0;
      }
    }
    if (!step_accepted) {
      break;
    }

    const double rotation_step = std::sqrt(next_x[0] * next_x[0] +
                                           next_x[1] * next_x[1] +
                                           next_x[2] * next_x[2]);
    const double translation_step =
        std::sqrt((next_x[3] - x[3]) * (next_x[3] - x[3]) +
                  (next_x[4] - x[4]) * (next_x[4] - x[4]) +
                  (next_x[5] - x[5]) * (next_x[5] - x[5]));
    const double cost_improvement = cost - next_cost;
    for (int k = 0; k < 6; ++k) {
      x[k] = next_x[k];
    }
    cost = next_cost;
    lambda = std::max(lambda * 0.3, 1.0e-12);
    if (cost < 1.0e-12 ||
        (cost_improvement < 1.0e-14 && rotation_step < 1.0e-8 &&
         translation_step < 1.0e-8 * translation_scale)) {
      converged = true;
      break;
    }
  }

  if (!std::isfinite(cost) || cost > start_cost * 4.0 + 1.0e-9) {
    return false;  // diverged -> keep the IPPE solution
  }
  *rvec_out = cv::Vec3d(x[0], x[1], x[2]);
  *tvec_out = cv::Vec3d(x[3], x[4], x[5]);
  *cost_out = cost;
  (void)converged;
  return true;
}

// One refined pose candidate.
struct Candidate {
  cv::Vec3d rvec{0, 0, 0};
  cv::Vec3d tvec{0, 0, 0};
  cv::Matx33d R = cv::Matx33d::eye();
  double cost = kInvalidCost;           // mean squared conic residual [px^2]
  double rmse = std::numeric_limits<double>::infinity();  // [px]
  double facing = 0.0;  // -n_z of the canonical (face-on) normal, 0..1
  bool ippe_only = true;
};

Candidate makeCandidate(const CameraModel& camera, const EllipseConic& conic,
                        const std::vector<cv::Vec3d>& circle_points,
                        int refine_iterations, const cv::Vec3d& rvec_init,
                        const cv::Vec3d& tvec_init) {
  Candidate candidate;
  candidate.rvec = rvec_init;
  candidate.tvec = tvec_init;
  candidate.R = canonicalFacingRotation(rotationFromRvec(rvec_init));
  candidate.rvec = rvecFromRotation(candidate.R);
  candidate.cost = evaluateResiduals(camera, conic, circle_points, candidate.R,
                                     candidate.tvec, nullptr);
  candidate.ippe_only = true;

  auto accept_refinement = [&](const cv::Vec3d& refined_rvec,
                               const cv::Vec3d& refined_tvec,
                               double refined_cost) {
    cv::Matx33d refined_R =
        canonicalFacingRotation(rotationFromRvec(refined_rvec));
    const cv::Vec3d canonical_rvec = rvecFromRotation(refined_R);
    const double canonical_cost = evaluateResiduals(
        camera, conic, circle_points, refined_R, refined_tvec, nullptr);
    if (std::isfinite(canonical_cost) && canonical_cost <= candidate.cost) {
      candidate.rvec = canonical_rvec;
      candidate.tvec = refined_tvec;
      candidate.R = refined_R;
      candidate.cost = canonical_cost;
      candidate.ippe_only = false;
      return true;
    }
    return false;
  };

  if (refine_iterations > 0) {
    cv::Vec3d refined_rvec;
    cv::Vec3d refined_tvec;
    double refined_cost = candidate.cost;
    if (refinePose(camera, conic, circle_points, refine_iterations,
                   candidate.rvec, candidate.tvec, &refined_rvec, &refined_tvec,
                   &refined_cost)) {
      accept_refinement(refined_rvec, refined_tvec, refined_cost);
    }
    // A converged circular-section refinement has an essentially zero residual
    // (any fitted ellipse is exactly realizable by a circle pose). If the LM
    // stalls prematurely - e.g. on a strongly blurred/elongated ellipse in a
    // fast-motion frame - restart it from the current pose with more
    // iterations. This is rare, so the average runtime is unaffected.
    constexpr double kStalledRefinementRmsePx = 0.02;
    if (!candidate.ippe_only &&
        std::sqrt(std::max(0.0, candidate.cost)) > kStalledRefinementRmsePx) {
      cv::Vec3d second_rvec;
      cv::Vec3d second_tvec;
      double second_cost = candidate.cost;
      const int second_iterations = std::max(2 * refine_iterations, 60);
      if (refinePose(camera, conic, circle_points, second_iterations,
                     candidate.rvec, candidate.tvec, &second_rvec, &second_tvec,
                     &second_cost)) {
        accept_refinement(second_rvec, second_tvec, second_cost);
      }
    }
  }

  candidate.rmse = std::sqrt(std::max(0.0, candidate.cost));
  const cv::Vec3d normal = candidate.R * cv::Vec3d(0.0, 0.0, 1.0);
  candidate.facing = -normal[2];
  return candidate;
}

// Two candidates describe the same physical disc pose (up to floating point).
bool samePose(const Candidate& a, const Candidate& b) {
  const cv::Vec3d normal_a = a.R * cv::Vec3d(0.0, 0.0, 1.0);
  const cv::Vec3d normal_b = b.R * cv::Vec3d(0.0, 0.0, 1.0);
  const double normal_dot = normal_a.dot(normal_b);
  const double scale = std::max(1.0, std::max(cv::norm(a.tvec), cv::norm(b.tvec)));
  const double translation_gap = cv::norm(a.tvec - b.tvec);
  const cv::Matx33d delta = a.R.t() * b.R;
  const bool same = normal_dot >= kSamePoseNormalDot &&
                    translation_gap <= kSamePoseTranslationAbs +
                                           kSamePoseTranslationRatio * scale &&
                    std::fabs(delta(2, 2) - 1.0) < 1.0e-3;
  GT_POSE_LOG(
      "[pose] samePose ndot=%.7f tdiff=%.4f d22=%.7f -> %d\n", normal_dot,
      translation_gap, delta(2, 2), same ? 1 : 0);
  return same;
}

// Chooses between the refined candidates.
// Returns the chosen index and sets *ambiguous when the choice is not backed by
// a clear facing / cost / temporal-continuity decision.
//
// Important property of circle PnP: *every* image ellipse is exactly the
// projection of two circles of the given radius (the two circular sections of
// the back-projection cone), so the conic reprojection cost of both solutions
// is essentially zero and cannot be used as the primary discriminator. The
// temporal prior (plus a facing sanity check) is the meaningful selector; the
// cost only vetos candidates whose refinement obviously failed.
int selectCandidate(const std::vector<Candidate>& candidates,
                    const cv::Matx33d* previous_R,
                    const cv::Vec3d* previous_tvec, double cost_margin,
                    bool ambiguity_resolution, bool* ambiguous) {
  *ambiguous = false;
  if (candidates.empty()) {
    return -1;
  }
  if (candidates.size() == 1) {
    return 0;
  }
  if (!ambiguity_resolution) {
    int best = 0;
    for (int i = 1; i < static_cast<int>(candidates.size()); ++i) {
      if (candidates[static_cast<size_t>(i)].rmse <
          candidates[static_cast<size_t>(best)].rmse) {
        best = i;
      }
    }
    return best;
  }

  // Collapse numerically identical candidates (e.g. the previous-pose seed and
  // an IPPE seed that converged to the same circular section).
  std::vector<int> unique;
  for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
    int duplicate = -1;
    for (size_t j = 0; j < unique.size(); ++j) {
      if (samePose(candidates[static_cast<size_t>(i)],
                   candidates[static_cast<size_t>(unique[j])])) {
        duplicate = static_cast<int>(j);
        break;
      }
    }
    if (duplicate < 0) {
      unique.push_back(i);
    } else if (candidates[static_cast<size_t>(i)].rmse <
               candidates[static_cast<size_t>(
                   unique[static_cast<size_t>(duplicate)])]
                   .rmse) {
      unique[static_cast<size_t>(duplicate)] = i;
    }
  }
  if (unique.size() == 1) {
    return unique[0];
  }

  // Drop candidates whose refinement clearly failed: with a converged circle
  // pose the conic residual is essentially zero, so a candidate that is much
  // worse than the best one must not win on temporal continuity alone.
  {
    double best_rmse = candidates[static_cast<size_t>(unique[0])].rmse;
    for (int index : unique) {
      best_rmse = std::min(best_rmse, candidates[static_cast<size_t>(index)].rmse);
    }
    const double fit_tolerance = std::max(0.05, 3.0 * best_rmse);
    std::vector<int> viable;
    for (int index : unique) {
      if (candidates[static_cast<size_t>(index)].rmse <=
          best_rmse + fit_tolerance) {
        viable.push_back(index);
      }
    }
    if (viable.size() < unique.size()) {
      unique.swap(viable);
      if (unique.size() == 1) {
        return unique[0];
      }
    }
  }

  // 1) facing sanity: the glowing disc is only visible from its front side, so
  //    a solution whose normal is almost parallel to the image plane is not a
  //    plausible target.
  std::vector<int> facing_candidates;
  for (int index : unique) {
    if (candidates[static_cast<size_t>(index)].facing >= kMinFacingNz) {
      facing_candidates.push_back(index);
    }
  }
  if (facing_candidates.size() == 1) {
    return facing_candidates[0];
  }
  if (facing_candidates.empty()) {
    *ambiguous = true;
    int best = unique[0];
    for (int index : unique) {
      if (candidates[static_cast<size_t>(index)].facing >
          candidates[static_cast<size_t>(best)].facing) {
        best = index;
      }
    }
    return best;
  }
  const std::vector<int>& pool = facing_candidates;

  // 2) reprojection cost: only meaningful when a refinement failed to reach an
  //    exact circular section (in that case its RMSE is well above zero).
  int lowest = pool[0];
  int second = -1;
  double lowest_rmse = candidates[static_cast<size_t>(lowest)].rmse;
  for (size_t i = 1; i < pool.size(); ++i) {
    const double rmse = candidates[static_cast<size_t>(pool[i])].rmse;
    if (rmse < lowest_rmse) {
      second = lowest;
      lowest = pool[i];
      lowest_rmse = rmse;
    } else if (second < 0 ||
               rmse < candidates[static_cast<size_t>(second)].rmse) {
      second = pool[i];
    }
  }
  constexpr double kMeaningfulCostPx = 0.05;
  if (second >= 0) {
    const double second_rmse = candidates[static_cast<size_t>(second)].rmse;
    if (second_rmse > kMeaningfulCostPx &&
        (second_rmse - lowest_rmse) / second_rmse >
            std::max(0.0, cost_margin)) {
      return lowest;
    }
  }

  // 3) temporal continuity: pick the candidate closest to the previously
  //    accepted pose (plane normal direction and target centre). For a smooth
  //    sequence this keeps the same circular section and prevents flips.
  if (previous_R != nullptr && previous_tvec != nullptr) {
    const cv::Vec3d previous_normal = *previous_R * cv::Vec3d(0.0, 0.0, 1.0);
    const double scale = std::max(1.0, cv::norm(*previous_tvec));
    int best = pool[0];
    double best_score = -std::numeric_limits<double>::infinity();
    double best_normal_dot = -1.0;
    for (int index : pool) {
      const cv::Vec3d normal =
          candidates[static_cast<size_t>(index)].R * cv::Vec3d(0.0, 0.0, 1.0);
      const double normal_dot = normal.dot(previous_normal);
      const double translation_gap =
          cv::norm(candidates[static_cast<size_t>(index)].tvec -
                   *previous_tvec) /
          scale;
      const double score = normal_dot - 2.0 * translation_gap;
      if (score > best_score) {
        best_score = score;
        best_normal_dot = normal_dot;
        best = index;
      }
    }
    // The candidate is only "trusted" when it is genuinely close to the
    // previous pose; otherwise the frame is reported as ambiguous.
    *ambiguous = best_normal_dot < 0.95;
    return best;
  }

  // 4) cold start with two exact solutions: fundamentally ambiguous. Prefer
  //    the solution whose disc normal points most directly at the camera (the
  //    glowing face is only visible from the front, and the target is used
  //    roughly face-on in this application); flag the pose as ambiguous.
  *ambiguous = true;
  int cold_best = lowest;
  double cold_best_score = -2.0;
  for (int index : pool) {
    const cv::Vec3d tvec = candidates[static_cast<size_t>(index)].tvec;
    const double range = cv::norm(tvec);
    if (!(range > 1.0e-6)) {
      continue;
    }
    const cv::Vec3d line_of_sight = tvec * (1.0 / range);
    const cv::Vec3d normal =
        candidates[static_cast<size_t>(index)].R * cv::Vec3d(0.0, 0.0, 1.0);
    const double face_on_score = -normal.dot(line_of_sight);  // 1 = face-on
    if (face_on_score > cold_best_score + 1.0e-12) {
      cold_best_score = face_on_score;
      cold_best = index;
    } else if (std::fabs(face_on_score - cold_best_score) <= 1.0e-12 &&
               candidates[static_cast<size_t>(index)].rmse <
                   candidates[static_cast<size_t>(cold_best)].rmse) {
      cold_best = index;
    }
  }
  return cold_best;
}

}  // namespace

// ---------------------------------------------------------------------------
// PoseEstimator
// ---------------------------------------------------------------------------
struct PoseEstimator::Impl {
  CameraIntrinsics camera;
  TargetGeometry target;
  PoseConfig config;
  CameraModel model;
  double radius_mm = 25.0;
  std::vector<cv::Vec3d> circle_points;

  bool has_previous = false;
  PoseEstimate previous;

  void resetTemporal() {
    has_previous = false;
    previous = PoseEstimate();
  }
};

PoseEstimator::PoseEstimator(const CameraIntrinsics& camera,
                             const TargetGeometry& target,
                             const PoseConfig& config)
    : impl_(new Impl()) {
  impl_->camera = camera;
  impl_->target = target;
  impl_->config = config;
  impl_->model = CameraModel::from(camera);
  impl_->radius_mm = 0.5 * target.disc_diameter_mm;
  impl_->circle_points = makeCircleSamples(impl_->radius_mm, kRefineSamples);
}

PoseEstimator::~PoseEstimator() = default;

void PoseEstimator::reset() { impl_->resetTemporal(); }

PoseEstimate PoseEstimator::estimate(const Detection& detection) {
  const ScopedTimer timer;
  PoseEstimate pose;
  if (!impl_->model.valid() || !(impl_->radius_mm > 0.0) ||
      !std::isfinite(impl_->radius_mm)) {
    return pose;
  }

  EllipseConic conic;
  if (!buildEllipseConic(detection.disc_ellipse, &conic)) {
    return pose;
  }
  // The ellipse must at least partially overlap the image.
  const double reach =
      2.0 * std::max(conic.a, conic.b) + 2.0 * kPi * 0.25 * conic.a;
  const double width = static_cast<double>(impl_->camera.width);
  const double height = static_cast<double>(impl_->camera.height);
  if (width > 0.0 && height > 0.0 &&
      (conic.center.x < -reach || conic.center.x > width + reach ||
       conic.center.y < -reach || conic.center.y > height + reach)) {
    return pose;
  }

  // --- affine-consistent initialization -----------------------------------
  // For a circle any in-plane rotation of the target is unobservable, so the
  // ellipse principal axes can be used directly as the projected target axes.
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  object_points.reserve(4);
  image_points.reserve(4);
  const cv::Point2d major_dir(std::cos(conic.theta), std::sin(conic.theta));
  const cv::Point2d minor_dir(-std::sin(conic.theta), std::cos(conic.theta));
  for (int i = 0; i < 4; ++i) {
    const double angle = 0.5 * kPi * static_cast<double>(i);
    object_points.emplace_back(
        static_cast<float>(impl_->radius_mm * std::cos(angle)),
        static_cast<float>(impl_->radius_mm * std::sin(angle)), 0.0f);
    const cv::Point2d point =
        conic.center + conic.a * std::cos(angle) * major_dir +
        conic.b * std::sin(angle) * minor_dir;
    image_points.emplace_back(static_cast<float>(point.x),
                              static_cast<float>(point.y));
  }

  const cv::Mat camera_matrix = impl_->camera.cameraMatrix();
  const cv::Mat distortion = impl_->camera.distortion();
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  int solution_count = 0;
  try {
    solution_count =
        cv::solvePnPGeneric(object_points, image_points, camera_matrix,
                            distortion, rvecs, tvecs, false, cv::SOLVEPNP_IPPE);
  } catch (const cv::Exception&) {
    solution_count = 0;
  }
  if (solution_count <= 0 || rvecs.empty()) {
    // Degenerate ellipse: fall back to a plain iterative PnP from the affine
    // initialization (this rarely happens for a real disc).
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    try {
      if (cv::solvePnP(object_points, image_points, camera_matrix, distortion,
                       rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
        rvecs.clear();
        tvecs.clear();
        rvecs.push_back(cv::Mat(rvec));
        tvecs.push_back(cv::Mat(tvec));
        solution_count = 1;
      }
    } catch (const cv::Exception&) {
      solution_count = 0;
    }
  }
  pose.num_solutions = solution_count;
  if (solution_count <= 0) {
    GT_POSE_LOG("[pose] IPPE failed (a=%.2f b=%.2f)\n", conic.a, conic.b);
    pose.solve_ms = timer.elapsedMs();
    return pose;
  }
  GT_POSE_LOG("[pose] ellipse c=(%.2f,%.2f) a=%.2f b=%.2f th=%.2f nsol=%d\n",
              conic.center.x, conic.center.y, conic.a, conic.b,
              conic.theta * kRad2Deg, solution_count);

  // --- refine every candidate ---------------------------------------------
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<size_t>(solution_count));
  for (int i = 0; i < solution_count; ++i) {
    const cv::Mat& rvec_mat = rvecs[static_cast<size_t>(i)];
    const cv::Mat& tvec_mat = tvecs[static_cast<size_t>(i)];
    if (rvec_mat.total() < 3 || tvec_mat.total() < 3) {
      continue;
    }
    const cv::Vec3d rvec(rvec_mat.at<double>(0, 0), rvec_mat.at<double>(1, 0),
                         rvec_mat.at<double>(2, 0));
    const cv::Vec3d tvec(tvec_mat.at<double>(0, 0), tvec_mat.at<double>(1, 0),
                         tvec_mat.at<double>(2, 0));
    if (!std::isfinite(rvec[0]) || !std::isfinite(rvec[1]) ||
        !std::isfinite(rvec[2]) || !std::isfinite(tvec[0]) ||
        !std::isfinite(tvec[1]) || !std::isfinite(tvec[2]) || tvec[2] <= 0.0) {
      continue;
    }
    candidates.push_back(makeCandidate(
        impl_->model, conic, impl_->circle_points, impl_->config.refine_iterations,
        rvec, tvec));
  }
  if (candidates.empty()) {
    GT_POSE_LOG("[pose] no finite candidate\n");
    pose.solve_ms = timer.elapsedMs();
    return pose;
  }
  for (size_t i = 0; i < candidates.size(); ++i) {
    const cv::Vec3d normal = candidates[i].R * cv::Vec3d(0, 0, 1);
    GT_POSE_LOG(
        "[pose] cand%zu rmse=%.5f t=(%.2f,%.2f,%.2f) n=(%.3f,%.3f,%.3f) "
        "facing=%.3f ippe_only=%d\n",
        i, candidates[i].rmse, candidates[i].tvec[0], candidates[i].tvec[1],
        candidates[i].tvec[2], normal[0], normal[1], normal[2],
        candidates[i].facing, candidates[i].ippe_only ? 1 : 0);
  }
  if (candidates.size() == 2) {
    const cv::Vec3d na = candidates[0].R * cv::Vec3d(0, 0, 1);
    const cv::Vec3d nb = candidates[1].R * cv::Vec3d(0, 0, 1);
    GT_POSE_LOG("[pose] normal dot %.5f tdiff %.3f same=%d\n", na.dot(nb),
                cv::norm(candidates[0].tvec - candidates[1].tvec),
                samePose(candidates[0], candidates[1]) ? 1 : 0);
  }

  // Temporal seed: refining from the previously accepted pose is the reliable
  // way to stay on the same one of the two circular sections. It can only be
  // added when the refinement actually converged (otherwise the raw previous
  // pose would pollute the candidate set).
  if (impl_->has_previous && impl_->config.refine_iterations > 0) {
    Candidate temporal =
        makeCandidate(impl_->model, conic, impl_->circle_points,
                      impl_->config.refine_iterations, impl_->previous.rvec,
                      impl_->previous.tvec);
    if (!temporal.ippe_only) {
      candidates.push_back(temporal);
      GT_POSE_LOG("[pose] temporal seed rmse=%.5f t=(%.2f,%.2f,%.2f)\n",
                  temporal.rmse, temporal.tvec[0], temporal.tvec[1],
                  temporal.tvec[2]);
    }
  }

  const bool has_previous = impl_->has_previous;
  const int chosen = selectCandidate(
      candidates, has_previous ? &impl_->previous.R : nullptr,
      has_previous ? &impl_->previous.tvec : nullptr,
      impl_->config.ambiguity_cost_margin, impl_->config.enable_ambiguity_resolution,
      &pose.ambiguous);
  if (chosen < 0) {
    pose.solve_ms = timer.elapsedMs();
    return pose;
  }

  Candidate& best = candidates[static_cast<size_t>(chosen)];
  // The canonical facing representation is required for a well-defined normal.
  best.R = canonicalFacingRotation(best.R);
  best.rvec = rvecFromRotation(best.R);
  // Always report the reprojection residual of the final pose, including when
  // the refinement failed and the raw IPPE solution is used.
  const double final_rmse = computeReprojRmse(
      impl_->model, conic, impl_->circle_points, best.R, best.tvec);
  if (std::isfinite(final_rmse)) {
    best.rmse = final_rmse;
    best.cost = final_rmse * final_rmse;
  }

  const cv::Vec3d tvec = best.tvec;
  pose.rvec = best.rvec;
  pose.tvec = tvec;
  pose.R = best.R;
  pose.reproj_rmse_px = best.rmse;
  pose.yaw = std::atan2(tvec[0], tvec[2]);
  pose.pitch = -std::atan2(tvec[1], tvec[2]);
  pose.distance_m = cv::norm(tvec) * 1.0e-3;
  pose.solve_ms = timer.elapsedMs();

  // --- validation -----------------------------------------------------------
  bool valid = true;
  if (!std::isfinite(pose.distance_m) || !(tvec[2] > 0.0)) {
    valid = false;
  }
  if (pose.distance_m < impl_->config.min_distance_m ||
      pose.distance_m > impl_->config.max_distance_m) {
    valid = false;
  }
  if (!std::isfinite(pose.reproj_rmse_px) ||
      pose.reproj_rmse_px > impl_->config.max_reproj_rmse_px) {
    valid = false;
  }
  GT_POSE_LOG(
      "[pose] chosen=%d amb=%d valid=%d d=%.3f rmse=%.4f yaw=%.3f pitch=%.3f\n",
      chosen, pose.ambiguous ? 1 : 0, valid ? 1 : 0, pose.distance_m,
      pose.reproj_rmse_px, pose.yaw * kRad2Deg, pose.pitch * kRad2Deg);
  pose.valid = valid;
  if (valid) {
    impl_->previous = pose;
    impl_->has_previous = true;
  }
  return pose;
}

void PoseEstimator::projectDisc(const PoseEstimate& pose, int samples,
                                std::vector<cv::Point2f>* points) const {
  if (points == nullptr) {
    return;
  }
  points->clear();
  if (!impl_->model.valid() || !(impl_->radius_mm > 0.0)) {
    return;
  }
  const int count = samples > 0 ? samples : kRefineSamples;
  const std::vector<cv::Vec3d> circle =
      makeCircleSamples(impl_->radius_mm, count);
  const cv::Matx33d R = rotationFromRvec(pose.rvec);
  points->reserve(circle.size());
  for (const cv::Vec3d& point : circle) {
    cv::Point2d projected;
    if (impl_->model.project(R, pose.tvec, point, &projected)) {
      points->emplace_back(static_cast<float>(projected.x),
                           static_cast<float>(projected.y));
    }
  }
}

}  // namespace glowtrack
