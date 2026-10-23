// Unit tests for glowtrack::AngleKalmanFilter / RangeKalmanFilter.
#include "test_framework.hpp"

#include "glowtrack/kalman_filter.hpp"
#include "glowtrack/types.hpp"
#include "glowtrack/util.hpp"

#include <algorithm>
#include <cmath>
#include <random>

using namespace glowtrack;

namespace {

constexpr double kDegToRadLocal = kPi / 180.0;

KalmanConfig defaultKalmanConfig() {
  KalmanConfig config;
  config.process_noise_angle = 2.0e-4;
  config.process_noise_rate = 2.0e-2;
  config.meas_sigma_deg = 0.6;
  config.meas_sigma_scale_px = 40.0;
  config.gate_chi2 = 13.8;
  config.max_prediction_frames = 15;
  config.quality_decay_per_frame = 0.9;
  config.range_process_noise = 1.0e-2;
  config.range_meas_sigma_m = 0.05;
  config.latency_compensation_ms = 0.0;
  return config;
}

constexpr double kDt = 1.0 / 30.0;

double measurementSigmaRad(const KalmanConfig& config) {
  return config.meas_sigma_deg * kDegToRadLocal;
}

// Runs a constant-velocity angle sequence through the filter and returns the
// final measurement time (the filter state corresponds to it).
void feedAngleConstantVelocity(AngleKalmanFilter* filter,
                               const KalmanConfig& config, double yaw0,
                               double pitch0, double yaw_rate,
                               double pitch_rate, int frames,
                               std::mt19937* rng) {
  std::normal_distribution<double> noise(0.0, measurementSigmaRad(config));
  for (int frame = 1; frame <= frames; ++frame) {
    const double t = frame * kDt;
    const double truth_yaw = normalizeAngle(yaw0 + yaw_rate * t);
    const double truth_pitch = pitch0 + pitch_rate * t;
    filter->predict(kDt);
    const bool accepted = filter->update(
        normalizeAngle(truth_yaw + noise(*rng)), truth_pitch + noise(*rng),
        measurementSigmaRad(config));
    GT_CHECK(accepted);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// Constant-velocity tracking convergence.
// ---------------------------------------------------------------------------
GT_TEST(kalman_angle_convergence) {
  const KalmanConfig config = defaultKalmanConfig();
  std::mt19937 rng(1234);
  const double yaw0 = 0.30;
  const double pitch0 = -0.20;
  const double yaw_rate = 0.8;
  const double pitch_rate = 0.4;
  AngleKalmanFilter filter(config);
  GT_CHECK(!filter.initialized());
  filter.initialize(yaw0, pitch0);
  GT_CHECK(filter.initialized());

  const int frames = 90;
  feedAngleConstantVelocity(&filter, config, yaw0, pitch0, yaw_rate,
                            pitch_rate, frames, &rng);

  const double t = frames * kDt;
  const double truth_yaw = normalizeAngle(yaw0 + yaw_rate * t);
  const double truth_pitch = pitch0 + pitch_rate * t;
  GT_CHECK_LT(std::fabs(angleDifference(filter.yaw(), truth_yaw)), 0.02);
  GT_CHECK_LT(std::fabs(filter.pitch() - truth_pitch), 0.02);
  GT_CHECK_LT(std::fabs(filter.yawRate() - yaw_rate), 0.20);
  GT_CHECK_LT(std::fabs(filter.pitchRate() - pitch_rate), 0.20);

  // A quarter-second extrapolation must stay on the true trajectory.
  double ahead_yaw = 0.0;
  double ahead_pitch = 0.0;
  filter.predictAhead(0.25, &ahead_yaw, &ahead_pitch, nullptr, nullptr);
  const double truth_yaw_ahead = normalizeAngle(yaw0 + yaw_rate * (t + 0.25));
  const double truth_pitch_ahead = pitch0 + pitch_rate * (t + 0.25);
  GT_CHECK_LT(std::fabs(angleDifference(ahead_yaw, truth_yaw_ahead)), 0.05);
  GT_CHECK_LT(std::fabs(ahead_pitch - truth_pitch_ahead), 0.05);
}

// ---------------------------------------------------------------------------
// Chi-square innovation gate rejects an outlier without touching the state.
// ---------------------------------------------------------------------------
GT_TEST(kalman_angle_outlier_gate) {
  const KalmanConfig config = defaultKalmanConfig();
  std::mt19937 rng(77);
  AngleKalmanFilter filter(config);
  filter.initialize(0.1, 0.05);
  feedAngleConstantVelocity(&filter, config, 0.1, 0.05, 0.3, -0.2, 40, &rng);

  const double yaw_before = filter.yaw();
  const double pitch_before = filter.pitch();
  const double yaw_rate_before = filter.yawRate();
  // A 30 degree jump is far outside the gate.
  GT_CHECK(!filter.update(yaw_before + 30.0 * kDegToRadLocal,
                          pitch_before + 20.0 * kDegToRadLocal,
                          measurementSigmaRad(config)));
  GT_CHECK_NEAR(filter.yaw(), yaw_before, 1e-12);
  GT_CHECK_NEAR(filter.pitch(), pitch_before, 1e-12);
  GT_CHECK_NEAR(filter.yawRate(), yaw_rate_before, 1e-12);

  // A consistent measurement is still accepted afterwards.
  const double t = 41 * kDt;
  const double truth_yaw = normalizeAngle(0.1 + 0.3 * t);
  const double truth_pitch = 0.05 - 0.2 * t;
  GT_CHECK(filter.update(truth_yaw, truth_pitch, measurementSigmaRad(config)));
}

// ---------------------------------------------------------------------------
// Yaw wrap-around across +-pi must be handled without a spike.
// ---------------------------------------------------------------------------
GT_TEST(kalman_angle_wrap_around) {
  const KalmanConfig config = defaultKalmanConfig();
  std::mt19937 rng(9);
  std::normal_distribution<double> noise(0.0, measurementSigmaRad(config));
  AngleKalmanFilter filter(config);
  const double yaw_rate = 1.2;
  // Start just below +pi so the trajectory crosses the wrap boundary.
  filter.initialize(kPi - 0.10, 0.0);

  double max_error = 0.0;
  const int frames = 60;
  for (int frame = 1; frame <= frames; ++frame) {
    const double t = frame * kDt;
    const double truth_yaw = normalizeAngle(kPi - 0.10 + yaw_rate * t);
    filter.predict(kDt);
    const bool accepted = filter.update(
        normalizeAngle(truth_yaw + noise(rng)), noise(rng),
        measurementSigmaRad(config));
    GT_CHECK(accepted);
    max_error = std::max(
        max_error, std::fabs(angleDifference(filter.yaw(), truth_yaw)));
  }
  GT_CHECK_LT(max_error, 0.05);  // < ~3 deg, no wrap spike
  GT_CHECK_LT(std::fabs(filter.yawRate() - yaw_rate), 0.25);
  // The state stays in the canonical (-pi, pi] range.
  GT_CHECK(filter.yaw() > -kPi && filter.yaw() <= kPi);

  // Residual wrapping on the model side: a measurement on the other side of
  // the boundary is close to the state, not a 2*pi outlier.
  AngleKalmanFilter boundary(config);
  boundary.initialize(kPi - 0.02, 0.0);
  GT_CHECK(boundary.update(-kPi + 0.02, 0.0, measurementSigmaRad(config)));
  GT_CHECK_LT(std::fabs(angleDifference(boundary.yaw(), -kPi + 0.02)), 0.2);
}

// ---------------------------------------------------------------------------
// Prediction through a detection gap: angles coast, covariance/quality decay.
// ---------------------------------------------------------------------------
GT_TEST(kalman_angle_prediction_gap_and_quality) {
  const KalmanConfig config = defaultKalmanConfig();
  std::mt19937 rng(4242);
  const double yaw0 = -0.4;
  const double pitch0 = 0.1;
  const double yaw_rate = 0.5;
  const double pitch_rate = -0.25;
  AngleKalmanFilter filter(config);
  filter.initialize(yaw0, pitch0);
  feedAngleConstantVelocity(&filter, config, yaw0, pitch0, yaw_rate,
                            pitch_rate, 45, &rng);

  const double quality_before = filter.quality();
  const double p00_before = filter.covariance()(0, 0);
  GT_CHECK_GT(quality_before, 0.5);

  const int gap_frames = 10;
  for (int frame = 0; frame < gap_frames; ++frame) {
    filter.predict(kDt);
  }
  const double quality_after = filter.quality();
  const double p00_after = filter.covariance()(0, 0);
  GT_CHECK_GT(p00_after, p00_before);       // uncertainty grows while coasting
  GT_CHECK_LT(quality_after, quality_before);  // ... and confidence decays
  GT_CHECK_LT(quality_after, 0.95);
  GT_CHECK_GT(quality_after, 0.0);

  // Extrapolation through the gap follows the constant-velocity truth.
  const double t = (45 + gap_frames) * kDt;
  double ahead_yaw = 0.0;
  double ahead_pitch = 0.0;
  filter.predictAhead(0.0, &ahead_yaw, &ahead_pitch, nullptr, nullptr);
  const double truth_yaw = normalizeAngle(yaw0 + yaw_rate * t);
  const double truth_pitch = pitch0 + pitch_rate * t;
  GT_CHECK_LT(std::fabs(angleDifference(ahead_yaw, truth_yaw)), 0.10);
  GT_CHECK_LT(std::fabs(ahead_pitch - truth_pitch), 0.10);

  // A measurement after the gap is accepted (gate scales with the grown
  // covariance) and restores confidence.
  GT_CHECK(filter.update(truth_yaw, truth_pitch, measurementSigmaRad(config)));
  GT_CHECK_GT(filter.quality(), quality_after);
}

// ---------------------------------------------------------------------------
// Measurement noise scaling with apparent target size.
// ---------------------------------------------------------------------------
GT_TEST(kalman_measurement_sigma_scaling) {
  KalmanConfig config = defaultKalmanConfig();
  config.meas_sigma_deg = 0.6;
  config.meas_sigma_scale_px = 40.0;
  AngleKalmanFilter filter(config);
  const double base = 0.6 * kDegToRadLocal;

  // Reference radius keeps the configured base noise.
  GT_CHECK_NEAR(filter.measurementSigmaRad(40.0), base, 1e-9);
  // Larger target (closer): clamped to the base noise.
  GT_CHECK_NEAR(filter.measurementSigmaRad(120.0), base, 1e-9);
  GT_CHECK_NEAR(filter.measurementSigmaRad(40.0), filter.measurementSigmaRad(200.0),
                1e-9);
  // Smaller target (farther): noise grows proportionally, clamped at 8x.
  GT_CHECK_NEAR(filter.measurementSigmaRad(10.0), 4.0 * base, 1e-9);
  GT_CHECK_NEAR(filter.measurementSigmaRad(1.0), 8.0 * base, 1e-9);
  GT_CHECK_NEAR(filter.measurementSigmaRad(0.0), 8.0 * base, 1e-9);

  // Non-positive sigma to update() falls back to the nominal configured noise.
  AngleKalmanFilter a(config);
  a.initialize(0.0, 0.0);
  GT_CHECK(a.update(0.0, 0.0, 0.0));
  AngleKalmanFilter b(config);
  b.initialize(0.0, 0.0);
  GT_CHECK(b.update(0.0, 0.0, base));
}

// ---------------------------------------------------------------------------
// Uninitialized filter behaviour and reset().
// ---------------------------------------------------------------------------
GT_TEST(kalman_angle_state_lifecycle) {
  const KalmanConfig config = defaultKalmanConfig();
  AngleKalmanFilter filter(config);
  GT_CHECK(!filter.initialized());
  GT_CHECK(!filter.update(0.1, 0.1, 0.01));  // not initialized -> rejected
  GT_CHECK_NEAR(filter.quality(), 0.0, 0.0);
  filter.predict(kDt);  // no-op before initialization
  GT_CHECK_NEAR(filter.yaw(), 0.0, 0.0);

  filter.initialize(0.2, -0.1);
  GT_CHECK(filter.initialized());
  GT_CHECK(filter.update(0.2, -0.1, 0.01));
  GT_CHECK_GT(filter.quality(), 0.5);
  GT_CHECK_LT(filter.quality(), 1.0 + 1e-12);

  filter.reset();
  GT_CHECK(!filter.initialized());
  GT_CHECK_NEAR(filter.yaw(), 0.0, 0.0);
  GT_CHECK_NEAR(filter.pitch(), 0.0, 0.0);
}

// ---------------------------------------------------------------------------
// Range filter: convergence, gating, prediction through a gap.
// ---------------------------------------------------------------------------
GT_TEST(kalman_range_filter) {
  const KalmanConfig config = defaultKalmanConfig();
  std::mt19937 rng(31337);
  std::normal_distribution<double> noise(0.0, config.range_meas_sigma_m);
  const double range0 = 0.8;
  const double range_rate = 0.15;  // m/s

  RangeKalmanFilter filter(config);
  GT_CHECK(!filter.initialized());
  GT_CHECK(!filter.update(range0, config.range_meas_sigma_m));
  filter.initialize(range0);
  GT_CHECK(filter.initialized());

  const int frames = 90;
  for (int frame = 1; frame <= frames; ++frame) {
    const double t = frame * kDt;
    filter.predict(kDt);
    GT_CHECK(filter.update(range0 + range_rate * t + noise(rng),
                           config.range_meas_sigma_m));
  }
  const double t = frames * kDt;
  const double truth_range = range0 + range_rate * t;
  GT_CHECK_LT(std::fabs(filter.range() - truth_range), 0.02);
  GT_CHECK_LT(std::fabs(filter.rate() - range_rate), 0.05);

  // Outlier spike is rejected and does not move the state.
  const double range_before = filter.range();
  GT_CHECK(!filter.update(1.8, config.range_meas_sigma_m));
  GT_CHECK_NEAR(filter.range(), range_before, 1e-12);

  // Prediction through a short gap advances along the estimated rate.
  const int gap_frames = 10;
  for (int frame = 0; frame < gap_frames; ++frame) {
    filter.predict(kDt);
  }
  GT_CHECK_NEAR(filter.range(), range_before + gap_frames * kDt * filter.rate(),
                0.01);
  const double gap_truth = range0 + range_rate * (t + gap_frames * kDt);
  GT_CHECK(filter.update(gap_truth, config.range_meas_sigma_m));
  GT_CHECK_LT(std::fabs(filter.range() - gap_truth), 0.03);

  filter.reset();
  GT_CHECK(!filter.initialized());
  GT_CHECK_NEAR(filter.range(), 0.0, 0.0);
}
