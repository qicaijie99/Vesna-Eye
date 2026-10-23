#pragma once
// Runtime configuration. The values are loaded from a YAML file through
// cv::FileStorage (OpenCV's YAML/JSON reader), so the tracker has no extra
// dependency besides OpenCV.

#include "glowtrack/types.hpp"

#include <string>

namespace glowtrack {

// ---------------------------------------------------------------------------
// Green-glow detector
// ---------------------------------------------------------------------------
struct DetectionConfig {
  // Green glow segmentation in OpenCV HSV space (H in [0,180], S/V in [0,255]).
  int green_h_min = 35;
  int green_h_max = 100;
  int green_s_min = 60;
  int green_v_min = 40;

  // Morphological cleaning of the green mask.
  int morph_open_ksize = 3;  // 0 = disable
  int morph_close_ksize = 5;

  // Search ROI: detector is only run inside this rect when a track is active.
  bool roi_search_enabled = true;
  float roi_padding_scale = 2.5f;  // pad = roi_padding_scale * predicted radius
  int roi_min_size_px = 80;

  // Candidate filtering (full-image search).
  int min_area_px = 200;
  int max_area_px = 400000;
  float min_axis_ratio = 0.45f;  // semi_minor / semi_major of the disc ellipse
  float max_axis_ratio = 1.0f;

  // White (saturated) center disc validation: the disc must be bright.
  int white_v_min = 200;
  int white_s_max = 60;

  // Green annulus validation around the disc: fraction of annulus samples that
  // must be green. scale = annulus radius / disc radius.
  float ring_inner_scale = 1.02f;
  float ring_outer_scale = 1.45f;
  int ring_samples = 64;
  float ring_green_min_ratio = 0.55f;

  // Confidence below this value is discarded.
  float score_threshold = 0.25f;

  // Fallback: detect the saturated white disc directly and require a green
  // annulus around it (used when the green ring is broken by motion blur).
  bool white_fallback_enabled = true;
};

// ---------------------------------------------------------------------------
// PnP / pose estimation (circle pose from an ellipse)
// ---------------------------------------------------------------------------
struct PoseConfig {
  double min_distance_m = 0.05;
  double max_distance_m = 20.0;
  double max_reproj_rmse_px = 4.0;  // reject poses with worse fit
  int refine_iterations = 20;       // Gauss-Newton refinement of the circle pose
  bool enable_ambiguity_resolution = true;
  double ambiguity_cost_margin = 0.10;  // relative cost margin to switch solution
};

// ---------------------------------------------------------------------------
// Kalman filter (LOS angles + rates, and range)
// ---------------------------------------------------------------------------
struct KalmanConfig {
  // Constant-velocity model on [yaw, pitch, yaw_rate, pitch_rate].
  double process_noise_angle = 2.0e-4;  // rad^2/s
  double process_noise_rate = 2.0e-2;   // rad^2/s^3
  // Measurement noise (1-sigma, degrees) scaled with apparent target size:
  // meas_sigma = base_deg * clamp(ref_radius_px / radius_px, 1, ...)
  double meas_sigma_deg = 0.6;
  double meas_sigma_scale_px = 40.0;
  double gate_chi2 = 13.8;  // ~2 dof, 99.9 %
  int max_prediction_frames = 15;
  double quality_decay_per_frame = 0.9;

  // Range filter (2 states: range, range_rate).
  double range_process_noise = 1.0e-2;
  double range_meas_sigma_m = 0.05;

  // Control latency compensation: output angles are predicted this far ahead.
  double latency_compensation_ms = 0.0;
};

// ---------------------------------------------------------------------------
// Tracker state machine / ROI handling
// ---------------------------------------------------------------------------
struct TrackerConfig {
  int acquire_hits = 3;        // consecutive detections before "tracking"
  int max_miss_frames = 15;    // prediction allowed before declaring lost
  int full_search_interval = 5;  // frames between full-frame searches when lost
  float prediction_growth_per_frame = 1.15f;
  int roi_min_size_px = 80;
};

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------
struct OutputConfig {
  bool csv_enabled = true;
  std::string csv_path = "results/track.csv";
  bool overlay_enabled = false;
  std::string overlay_video_path = "results/overlay.mp4";
  bool serial_enabled = false;
  std::string serial_port = "COM3";
  int serial_baud = 115200;
};

// ---------------------------------------------------------------------------
// Complete runtime configuration
// ---------------------------------------------------------------------------
struct RuntimeConfig {
  CameraIntrinsics camera;
  TargetGeometry target;
  DetectionConfig detector;
  PoseConfig pose;
  KalmanConfig kalman;
  TrackerConfig tracker;
  OutputConfig output;
  std::string source_path;  // loaded from YAML, may be overridden by CLI
  double nominal_fps = 30.0;
};

// Loads a YAML config file; throws std::runtime_error with a readable message
// on failure. Missing keys keep their default value.
RuntimeConfig loadRuntimeConfig(const std::string& path);

}  // namespace glowtrack
