#pragma once
// Kalman filters used by the tracker.
//
// AngleKalmanFilter
//   Constant-velocity model on the line-of-sight angles:
//     state = [yaw, pitch, yaw_rate, pitch_rate]
//   * yaw residual is wrapped to (-pi, pi],
//   * measurement noise grows when the target is small (few pixels),
//   * chi-square innovation gating rejects outliers,
//   * quality() decays when the filter runs without measurements.
//
// RangeKalmanFilter
//   Two-state [range, range_rate] filter for the PnP distance output.

#include "glowtrack/config.hpp"
#include "glowtrack/types.hpp"

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>

namespace glowtrack {

class AngleKalmanFilter {
 public:
  explicit AngleKalmanFilter(const KalmanConfig& config);

  void reset();
  void initialize(double yaw, double pitch);

  // Predicts the state forward by dt seconds (dt <= 0 is ignored).
  void predict(double dt);

  // Measurement update. Returns false if the measurement was rejected by the
  // innovation gate (or the filter is not initialized). A non-positive
  // meas_sigma_rad selects the configured nominal measurement noise.
  bool update(double yaw, double pitch, double meas_sigma_rad);

  // Measurement 1-sigma [rad] for a disc whose fitted ellipse has the given
  // mean radius in pixels: the configured base noise (meas_sigma_deg) is
  // scaled up for small (far) targets by meas_sigma_scale_px / radius.
  double measurementSigmaRad(double mean_radius_px) const;

  bool initialized() const { return initialized_; }

  double yaw() const;
  double pitch() const;
  double yawRate() const;
  double pitchRate() const;

  // 0..1 confidence based on the state covariance (1 = excellent).
  double quality() const;

  // Predict the angles `lead_s` seconds ahead without modifying the state.
  void predictAhead(double lead_s, double* yaw, double* pitch,
                    double* yaw_rate, double* pitch_rate) const;

  const cv::Matx44d& covariance() const;

 private:
  KalmanConfig config_;
  cv::Matx44d P_ = cv::Matx44d::eye();  // state covariance
  cv::Vec4d x_{0, 0, 0, 0};             // yaw, pitch, yaw_rate, pitch_rate
  bool initialized_ = false;
  // Innovation-adaptive process-noise boost used for maneuver handling: large
  // (but still accepted) innovations inflate the process noise of the next
  // predict steps and then decay back to 1.
  double maneuver_scale_ = 1.0;
};

class RangeKalmanFilter {
 public:
  explicit RangeKalmanFilter(const KalmanConfig& config);

  void reset();
  void initialize(double range_m);
  void predict(double dt);
  bool update(double range_m, double meas_sigma_m);

  bool initialized() const { return initialized_; }
  double range() const;
  double rate() const;

 private:
  KalmanConfig config_;
  cv::Matx22d P_ = cv::Matx22d::eye();
  cv::Vec2d x_{0.0, 0.0};  // range, range_rate
  bool initialized_ = false;
};

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), used by the serial protocol.
uint16_t crc16Ccitt(const uint8_t* data, size_t length);

}  // namespace glowtrack
