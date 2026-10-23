// TEMPORARY STUB -- owned by the pose/filter workstream.
#include "glowtrack/kalman_filter.hpp"

namespace glowtrack {

AngleKalmanFilter::AngleKalmanFilter(const KalmanConfig& config)
    : config_(config) {}

void AngleKalmanFilter::reset() {
  P_ = cv::Matx44d::eye();
  x_ = cv::Vec4d(0, 0, 0, 0);
  initialized_ = false;
}

void AngleKalmanFilter::initialize(double yaw, double pitch) {
  x_ = cv::Vec4d(yaw, pitch, 0, 0);
  P_ = cv::Matx44d::eye();
  initialized_ = true;
}

void AngleKalmanFilter::predict(double dt) {
  if (!initialized_ || dt <= 0.0) {
    return;
  }
  (void)dt;
}

bool AngleKalmanFilter::update(double yaw, double pitch, double meas_sigma_rad) {
  (void)yaw;
  (void)pitch;
  (void)meas_sigma_rad;
  return false;
}

double AngleKalmanFilter::yaw() const { return x_[0]; }
double AngleKalmanFilter::pitch() const { return x_[1]; }
double AngleKalmanFilter::yawRate() const { return x_[2]; }
double AngleKalmanFilter::pitchRate() const { return x_[3]; }

double AngleKalmanFilter::quality() const { return 0.0; }

void AngleKalmanFilter::predictAhead(double lead_s, double* yaw, double* pitch,
                                     double* yaw_rate, double* pitch_rate) const {
  (void)lead_s;
  if (yaw != nullptr) *yaw = x_[0];
  if (pitch != nullptr) *pitch = x_[1];
  if (yaw_rate != nullptr) *yaw_rate = x_[2];
  if (pitch_rate != nullptr) *pitch_rate = x_[3];
}

const cv::Matx44d& AngleKalmanFilter::covariance() const { return P_; }

RangeKalmanFilter::RangeKalmanFilter(const KalmanConfig& config)
    : config_(config) {}

void RangeKalmanFilter::reset() {
  P_ = cv::Matx22d::eye();
  x_ = cv::Vec2d(0, 0);
  initialized_ = false;
}

void RangeKalmanFilter::initialize(double range_m) {
  x_ = cv::Vec2d(range_m, 0.0);
  P_ = cv::Matx22d::eye();
  initialized_ = true;
}

void RangeKalmanFilter::predict(double dt) {
  if (!initialized_ || dt <= 0.0) {
    return;
  }
  (void)dt;
}

bool RangeKalmanFilter::update(double range_m, double meas_sigma_m) {
  (void)range_m;
  (void)meas_sigma_m;
  return false;
}

double RangeKalmanFilter::range() const { return x_[0]; }
double RangeKalmanFilter::rate() const { return x_[1]; }

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
