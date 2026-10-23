// Kalman filters used by the tracker.
//
// AngleKalmanFilter: 4-state constant-velocity model
//   x = [yaw, pitch, yaw_rate, pitch_rate]
// driven by a discrete white-noise acceleration model. The yaw residual is
// wrapped with normalizeAngle(), a chi-square test on the 2-DOF innovation
// rejects outliers and the reported quality decays while the filter coasts
// through detection gaps.
//
// RangeKalmanFilter: 2-state [range, range_rate] filter for the PnP distance.
//
// The CRC-16/CCITT-FALSE helper used by the serial protocol also lives here.

#include "glowtrack/kalman_filter.hpp"

#include "glowtrack/util.hpp"

#include <algorithm>
#include <cmath>

namespace glowtrack {
namespace {

// Diagonal covariance floor: keeps the matrices positive definite even after
// long sequences of predictions / updates.
constexpr double kCovarianceFloor = 1.0e-12;

// Angle standard deviation [rad] at which quality() has decayed to ~0.61.
constexpr double kQualitySigmaRef = 0.025;

// Maneuver adaptation: an accepted innovation that is close to the chi-square
// gate boosts the process noise of the following predict steps by up to
// (1 + kManeuverGain). This makes the filter catch fast target reversals
// without making it jittery in steady state (where innovations are small).
constexpr double kManeuverGain = 6.0;
constexpr double kManeuverDecay = 0.5;

// Longest prediction step fed to the process model; longer gaps (e.g. a video
// hiccup) saturate instead of exploding the covariance.
constexpr double kMaxPredictDt = 1.0;

double clampPositive(double value, double fallback) {
  if (!std::isfinite(value) || value <= 0.0) {
    return fallback;
  }
  return value;
}

void symmetrize(cv::Matx44d* matrix) {
  for (int row = 0; row < 4; ++row) {
    for (int col = row + 1; col < 4; ++col) {
      const double average =
          0.5 * ((*matrix)(row, col) + (*matrix)(col, row));
      (*matrix)(row, col) = average;
      (*matrix)(col, row) = average;
    }
  }
}

void symmetrize(cv::Matx22d* matrix) {
  const double average = 0.5 * ((*matrix)(0, 1) + (*matrix)(1, 0));
  (*matrix)(0, 1) = average;
  (*matrix)(1, 0) = average;
}

// Per-axis process noise of the constant-velocity model: random walk on the
// angle (config.process_noise_angle) plus white acceleration noise
// (config.process_noise_rate).
struct CvProcessNoise {
  double angle_variance = 0.0;
  double angle_rate_covariance = 0.0;
  double rate_variance = 0.0;
};

CvProcessNoise makeCvProcessNoise(double q_angle, double q_rate, double dt) {
  CvProcessNoise noise;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  noise.angle_variance = q_angle * dt + q_rate * dt3 / 3.0;
  noise.angle_rate_covariance = q_rate * dt2 / 2.0;
  noise.rate_variance = q_rate * dt;
  return noise;
}

}  // namespace

// ---------------------------------------------------------------------------
// AngleKalmanFilter
// ---------------------------------------------------------------------------
AngleKalmanFilter::AngleKalmanFilter(const KalmanConfig& config)
    : config_(config) {
  reset();
}

void AngleKalmanFilter::reset() {
  P_ = cv::Matx44d::eye();
  x_ = cv::Vec4d(0.0, 0.0, 0.0, 0.0);
  initialized_ = false;
  maneuver_scale_ = 1.0;
}

void AngleKalmanFilter::initialize(double yaw, double pitch) {
  x_ = cv::Vec4d(normalizeAngle(yaw), pitch, 0.0, 0.0);
  // After a single measurement the angles are known to roughly the measurement
  // noise; the rates are unknown but bounded by plausible target motion.
  const double sigma_angle =
      std::max(config_.meas_sigma_deg * kDeg2Rad, 0.5 * kDeg2Rad);
  const double sigma_rate = 0.5;  // rad/s
  P_ = cv::Matx44d::zeros();
  P_(0, 0) = sigma_angle * sigma_angle;
  P_(1, 1) = sigma_angle * sigma_angle;
  P_(2, 2) = sigma_rate * sigma_rate;
  P_(3, 3) = sigma_rate * sigma_rate;
  maneuver_scale_ = 1.0;
  initialized_ = true;
}

void AngleKalmanFilter::predict(double dt) {
  if (!initialized_ || !(dt > 0.0)) {
    return;
  }
  if (dt > kMaxPredictDt) {
    dt = kMaxPredictDt;
  }
  // Deterministic part of the constant-velocity model.
  x_[0] = normalizeAngle(x_[0] + dt * x_[2]);
  x_[1] = x_[1] + dt * x_[3];

  // P = F * P * F^T + Q with F = [I, dt*I; 0, I].
  cv::Matx44d FP;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      FP(row, col) = P_(row, col);
      if (row < 2) {
        FP(row, col) += dt * P_(row + 2, col);
      }
    }
  }
  cv::Matx44d predicted;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      predicted(row, col) = FP(row, col);
      if (col < 2) {
        predicted(row, col) += dt * FP(row, col + 2);
      }
    }
  }

  const double q_angle =
      std::max(0.0, config_.process_noise_angle) * maneuver_scale_;
  const double q_rate =
      std::max(0.0, config_.process_noise_rate) * maneuver_scale_;
  const CvProcessNoise noise = makeCvProcessNoise(q_angle, q_rate, dt);
  for (int axis = 0; axis < 2; ++axis) {
    const int angle_index = axis;
    const int rate_index = axis + 2;
    predicted(angle_index, angle_index) += noise.angle_variance;
    predicted(angle_index, rate_index) += noise.angle_rate_covariance;
    predicted(rate_index, angle_index) += noise.angle_rate_covariance;
    predicted(rate_index, rate_index) += noise.rate_variance;
  }
  symmetrize(&predicted);
  P_ = predicted;
  // The maneuver boost is transient: decay it back to the nominal noise level
  // over the next few predict steps.
  maneuver_scale_ = 1.0 + (maneuver_scale_ - 1.0) * kManeuverDecay;
}

bool AngleKalmanFilter::update(double yaw, double pitch,
                               double meas_sigma_rad) {
  if (!initialized_) {
    return false;
  }
  const double nominal_sigma = clampPositive(
      config_.meas_sigma_deg * kDeg2Rad, 1.0 * kDeg2Rad);
  const double sigma = clampPositive(meas_sigma_rad, nominal_sigma);
  const double variance = sigma * sigma;

  // Innovation, with the yaw residual wrapped to (-pi, pi].
  const double residual_yaw = normalizeAngle(yaw - x_[0]);
  const double residual_pitch = normalizeAngle(pitch - x_[1]);

  // S = H P H^T + R (2x2).
  const double s00 = P_(0, 0) + variance;
  const double s01 = P_(0, 1);
  const double s10 = P_(1, 0);
  const double s11 = P_(1, 1) + variance;
  const double determinant = s00 * s11 - s01 * s10;
  if (!(std::fabs(determinant) > 1.0e-18)) {
    return false;
  }
  const double inverse_det = 1.0 / determinant;

  // Chi-square innovation gate (2 degrees of freedom).
  const double gate = config_.gate_chi2;
  if (gate > 0.0) {
    const double distance_squared =
        (residual_yaw * (s11 * residual_yaw - s01 * residual_pitch) +
         residual_pitch * (s00 * residual_pitch - s10 * residual_yaw)) *
        inverse_det;
    if (!(distance_squared <= gate)) {
      return false;
    }
    // Accepted but unusually large innovation -> likely maneuver: boost the
    // process noise used by the following predictions so the rate state can
    // follow the target instead of lagging behind and overshooting later.
    const double ratio = std::min(1.0, distance_squared / gate);
    maneuver_scale_ = 1.0 + kManeuverGain * ratio * ratio;
  }

  // Kalman gain K = P H^T S^-1 (4x2); P H^T is the first two columns of P.
  double gain[4][2];
  for (int i = 0; i < 4; ++i) {
    const double c0 = P_(i, 0);
    const double c1 = P_(i, 1);
    gain[i][0] = (c0 * s11 - c1 * s10) * inverse_det;
    gain[i][1] = (-c0 * s01 + c1 * s00) * inverse_det;
  }

  for (int i = 0; i < 4; ++i) {
    x_[i] += gain[i][0] * residual_yaw + gain[i][1] * residual_pitch;
  }

  // P = P - K H P, with H P the first two rows of P.
  cv::Matx44d updated = P_;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      updated(row, col) -=
          gain[row][0] * P_(0, col) + gain[row][1] * P_(1, col);
    }
  }
  symmetrize(&updated);
  for (int i = 0; i < 4; ++i) {
    if (!std::isfinite(updated(i, i))) {
      return false;
    }
    updated(i, i) = std::max(updated(i, i), kCovarianceFloor);
  }
  P_ = updated;
  x_[0] = normalizeAngle(x_[0]);
  return true;
}

double AngleKalmanFilter::yaw() const { return x_[0]; }
double AngleKalmanFilter::pitch() const { return x_[1]; }
double AngleKalmanFilter::yawRate() const { return x_[2]; }
double AngleKalmanFilter::pitchRate() const { return x_[3]; }

double AngleKalmanFilter::quality() const {
  if (!initialized_) {
    return 0.0;
  }
  const double angle_variance = 0.5 * std::max(0.0, P_(0, 0) + P_(1, 1));
  const double sigma = std::sqrt(angle_variance);
  const double ratio = sigma / kQualitySigmaRef;
  const double quality = std::exp(-0.5 * ratio * ratio);
  return std::min(1.0, std::max(0.0, quality));
}

void AngleKalmanFilter::predictAhead(double lead_s, double* yaw, double* pitch,
                                     double* yaw_rate,
                                     double* pitch_rate) const {
  if (yaw != nullptr) {
    *yaw = normalizeAngle(x_[0] + lead_s * x_[2]);
  }
  if (pitch != nullptr) {
    *pitch = x_[1] + lead_s * x_[3];
  }
  if (yaw_rate != nullptr) {
    *yaw_rate = x_[2];
  }
  if (pitch_rate != nullptr) {
    *pitch_rate = x_[3];
  }
}

const cv::Matx44d& AngleKalmanFilter::covariance() const { return P_; }

double AngleKalmanFilter::measurementSigmaRad(double mean_radius_px) const {
  const double base =
      clampPositive(config_.meas_sigma_deg * kDeg2Rad, 1.0 * kDeg2Rad);
  double ratio = 1.0;
  const double scale_px = config_.meas_sigma_scale_px;
  if (scale_px > 0.0) {
    if (!(mean_radius_px > 1.0e-6)) {
      // No usable size estimate: be conservative.
      ratio = 8.0;
    } else {
      // Large (close) targets keep the base noise, small (far) targets get a
      // proportionally larger angular noise. The ratio is clamped so a spurious
      // tiny ellipse cannot destroy the update entirely.
      ratio = std::min(std::max(scale_px / mean_radius_px, 1.0), 8.0);
    }
  }
  return base * ratio;
}

// ---------------------------------------------------------------------------
// RangeKalmanFilter
// ---------------------------------------------------------------------------
RangeKalmanFilter::RangeKalmanFilter(const KalmanConfig& config)
    : config_(config) {
  reset();
}

void RangeKalmanFilter::reset() {
  P_ = cv::Matx22d::eye();
  x_ = cv::Vec2d(0.0, 0.0);
  initialized_ = false;
}

void RangeKalmanFilter::initialize(double range_m) {
  if (!std::isfinite(range_m)) {
    return;
  }
  x_ = cv::Vec2d(range_m, 0.0);
  const double sigma_range =
      clampPositive(config_.range_meas_sigma_m, 0.05);
  const double sigma_rate = 0.5;  // m/s
  P_ = cv::Matx22d::zeros();
  P_(0, 0) = sigma_range * sigma_range;
  P_(1, 1) = sigma_rate * sigma_rate;
  initialized_ = true;
}

void RangeKalmanFilter::predict(double dt) {
  if (!initialized_ || !(dt > 0.0)) {
    return;
  }
  if (dt > kMaxPredictDt) {
    dt = kMaxPredictDt;
  }
  x_[0] += dt * x_[1];
  const double q = std::max(0.0, config_.range_process_noise);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  // P = F P F^T + Q with F = [1, dt; 0, 1].
  cv::Matx22d predicted;
  predicted(0, 0) = P_(0, 0) + dt * (P_(0, 1) + P_(1, 0)) + dt2 * P_(1, 1) +
                    q * dt3 / 3.0;
  predicted(0, 1) = P_(0, 1) + dt * P_(1, 1) + q * dt2 / 2.0;
  predicted(1, 0) = P_(1, 0) + dt * P_(1, 1) + q * dt2 / 2.0;
  predicted(1, 1) = P_(1, 1) + q * dt;
  symmetrize(&predicted);
  P_ = predicted;
}

bool RangeKalmanFilter::update(double range_m, double meas_sigma_m) {
  if (!initialized_ || !std::isfinite(range_m) || !(range_m > 0.0)) {
    return false;
  }
  const double nominal_sigma =
      clampPositive(config_.range_meas_sigma_m, 0.05);
  const double sigma = clampPositive(meas_sigma_m, nominal_sigma);
  const double variance = sigma * sigma;

  const double residual = range_m - x_[0];
  const double innovation_variance = P_(0, 0) + variance;
  if (!(innovation_variance > 1.0e-12)) {
    return false;
  }
  // 1-DOF innovation gate. The configured gate is tuned for 2 DOF, so be a
  // little more permissive here (3-sigma equivalent) but still reject spikes.
  const double gate = std::max(9.0, config_.gate_chi2);
  if (residual * residual / innovation_variance > gate) {
    return false;
  }

  const double gain_range = P_(0, 0) / innovation_variance;
  const double gain_rate = P_(1, 0) / innovation_variance;
  x_[0] += gain_range * residual;
  x_[1] += gain_rate * residual;

  cv::Matx22d updated = P_;
  updated(0, 0) -= gain_range * P_(0, 0);
  updated(0, 1) -= gain_range * P_(0, 1);
  updated(1, 0) -= gain_rate * P_(0, 0);
  updated(1, 1) -= gain_rate * P_(0, 1);
  symmetrize(&updated);
  updated(0, 0) = std::max(updated(0, 0), kCovarianceFloor);
  updated(1, 1) = std::max(updated(1, 1), kCovarianceFloor);
  if (!std::isfinite(updated(0, 0)) || !std::isfinite(updated(1, 1))) {
    return false;
  }
  P_ = updated;
  return true;
}

double RangeKalmanFilter::range() const { return x_[0]; }
double RangeKalmanFilter::rate() const { return x_[1]; }

// ---------------------------------------------------------------------------
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), used by the serial protocol.
// ---------------------------------------------------------------------------
uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

}  // namespace glowtrack
