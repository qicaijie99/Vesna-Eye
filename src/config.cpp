// Runtime configuration loader: cv::FileStorage YAML -> RuntimeConfig.
//
// The YAML schema is documented in config/*.yaml and docs/architecture.md:
//
//   camera:    width, height, fx, fy, cx, cy, dist[], calibration_file?
//   target:    disc_diameter_mm, glow_outer_diameter_mm
//   detector:  ... (see DetectionConfig)
//   pose:      ... (see PoseConfig)
//   kalman:    ... (see KalmanConfig)
//   tracker:   ... (see TrackerConfig)
//   output:    csv_enabled, csv_path, overlay_enabled, overlay_video_path,
//              serial_enabled, serial_port, serial_baud
//   source:    optional default frame source path
//   nominal_fps: default frame rate
//
// Missing keys keep the C++ defaults. A missing / unreadable file throws
// std::runtime_error with a readable message.
//
// Optional camera.calibration_file points at a YAML file written by
// `glowtrack --mode calibrate` (keys image_width, image_height, camera_matrix,
// distortion_coefficients); it overrides the corresponding camera fields.

#include "glowtrack/config.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace glowtrack {
namespace {

// Returns the named mapping node or an empty node when it is missing / not a map.
cv::FileNode mapSection(const cv::FileNode& root, const char* name) {
  if (root.isNone()) {
    return cv::FileNode();
  }
  const cv::FileNode node = root[name];
  if (node.isNone() || !node.isMap()) {
    return cv::FileNode();
  }
  return node;
}

std::runtime_error configError(const std::string& path, const std::string& key,
                               const std::string& detail) {
  return std::runtime_error("config '" + path + "': cannot read '" + key +
                            "': " + detail);
}

std::string toLowerCopy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

// Generic scalar reader; leaves *value untouched when the key is missing.
template <typename T>
void readScalar(const std::string& path, const cv::FileNode& section,
                const char* key, T* value) {
  if (value == nullptr || section.isNone()) {
    return;
  }
  const cv::FileNode node = section[key];
  if (node.isNone()) {
    return;
  }
  try {
    node >> *value;
  } catch (const cv::Exception& e) {
    throw configError(path, key, e.what());
  }
}

// bool: accept real booleans (written as 1/0 by cv::FileStorage) and the usual
// textual spellings ("true"/"false"/"on"/"off"/"yes"/"no").
void readBool(const std::string& path, const cv::FileNode& section,
              const char* key, bool* value) {
  if (value == nullptr || section.isNone()) {
    return;
  }
  const cv::FileNode node = section[key];
  if (node.isNone()) {
    return;
  }
  try {
    if (node.isString()) {
      std::string text;
      node >> text;
      text = toLowerCopy(text);
      *value = (text == "true" || text == "1" || text == "yes" ||
                text == "on" || text == "enable" || text == "enabled");
    } else {
      int numeric = 0;
      node >> numeric;
      *value = numeric != 0;
    }
  } catch (const cv::Exception& e) {
    throw configError(path, key, e.what());
  }
}

// Sequence of doubles, e.g. dist: [0.0, 0.0, 0.0, 0.0, 0.0].
void readDoubleSequence(const std::string& path, const cv::FileNode& section,
                        const char* key, std::vector<double>* values) {
  if (values == nullptr || section.isNone()) {
    return;
  }
  const cv::FileNode node = section[key];
  if (node.isNone()) {
    return;
  }
  std::vector<double> parsed;
  try {
    if (node.isSeq()) {
      for (auto it = node.begin(); it != node.end(); ++it) {
        const cv::FileNode item = *it;
        double value = 0.0;
        item >> value;
        parsed.push_back(value);
      }
    } else {
      // Single scalar accepted for convenience.
      double value = 0.0;
      node >> value;
      parsed.push_back(value);
    }
  } catch (const cv::Exception& e) {
    throw configError(path, key, e.what());
  }
  if (!parsed.empty()) {
    *values = std::move(parsed);
  }
}

// Loads a calibration YAML produced by `glowtrack --mode calibrate` and
// overrides the camera model fields. Relative calibration paths are resolved
// first against the working directory (as given) and then against the
// directory of the tracking config file.
void applyCameraCalibrationFile(const std::string& config_path,
                                const std::string& calibration_file,
                                CameraIntrinsics* camera) {
  namespace fs = std::filesystem;
  if (camera == nullptr || calibration_file.empty()) {
    return;
  }
  fs::path calibration_path(calibration_file);
  if (calibration_path.is_relative()) {
    const fs::path config_dir = fs::path(config_path).parent_path();
    if (!config_dir.empty()) {
      const fs::path candidate = config_dir / calibration_path;
      if (fs::exists(candidate)) {
        calibration_path = candidate;
      }
    }
  }
  if (!fs::exists(calibration_path)) {
    throw std::runtime_error("config '" + config_path +
                             "': camera.calibration_file not found: '" +
                             calibration_path.string() + "'");
  }

  cv::FileStorage storage;
  bool opened = false;
  try {
    opened = storage.open(calibration_path.string(), cv::FileStorage::READ);
  } catch (const cv::Exception&) {
    opened = false;
  }
  if (!opened) {
    throw std::runtime_error("config '" + config_path +
                             "': cannot open camera.calibration_file '" +
                             calibration_path.string() + "'");
  }

  try {
    const cv::FileNode root = storage.root();
    if (root.isNone()) {
      throw std::runtime_error("calibration file is empty");
    }
    int width = 0;
    int height = 0;
    const cv::FileNode width_node = root["image_width"];
    const cv::FileNode height_node = root["image_height"];
    if (!width_node.isNone()) {
      width_node >> width;
    }
    if (!height_node.isNone()) {
      height_node >> height;
    }
    if (width > 0) {
      camera->width = width;
    }
    if (height > 0) {
      camera->height = height;
    }

    const cv::FileNode matrix_node = root["camera_matrix"];
    if (!matrix_node.isNone()) {
      cv::Mat matrix;
      matrix_node >> matrix;
      if (matrix.rows == 3 && matrix.cols == 3) {
        cv::Mat matrix64;
        matrix.convertTo(matrix64, CV_64F);
        camera->fx = matrix64.at<double>(0, 0);
        camera->fy = matrix64.at<double>(1, 1);
        camera->cx = matrix64.at<double>(0, 2);
        camera->cy = matrix64.at<double>(1, 2);
      } else {
        throw std::runtime_error("camera_matrix must be 3x3");
      }
    }

    const cv::FileNode dist_node = root["distortion_coefficients"];
    if (!dist_node.isNone()) {
      cv::Mat dist;
      dist_node >> dist;
      if (!dist.empty()) {
        cv::Mat dist64;
        dist.convertTo(dist64, CV_64F);
        dist64 = dist64.reshape(1, 1);  // 1 x N, continuous
        const int count = static_cast<int>(dist64.total());
        std::vector<double> coefficients;
        coefficients.reserve(static_cast<size_t>(count));
        for (int i = 0; i < count; ++i) {
          coefficients.push_back(dist64.at<double>(0, i));
        }
        camera->dist = std::move(coefficients);
      }
    }
  } catch (const cv::Exception& e) {
    throw std::runtime_error("config '" + config_path +
                             "': invalid camera calibration file '" +
                             calibration_path.string() + "': " + e.what());
  } catch (const std::runtime_error& e) {
    throw std::runtime_error("config '" + config_path +
                             "': invalid camera calibration file '" +
                             calibration_path.string() + "': " + e.what());
  }
}

}  // namespace

RuntimeConfig loadRuntimeConfig(const std::string& path) {
  RuntimeConfig config;

  cv::FileStorage storage;
  bool opened = false;
  try {
    opened = storage.open(path, cv::FileStorage::READ);
  } catch (const cv::Exception&) {
    opened = false;
  }
  if (!opened) {
    throw std::runtime_error("cannot open config file '" + path +
                             "' (cv::FileStorage failed to parse it)");
  }

  std::string calibration_file;
  try {
    const cv::FileNode root = storage.root();
    if (root.isNone()) {
      throw std::runtime_error("config file '" + path + "' is empty");
    }

    // ------------------------------------------------------------------
    // camera
    // ------------------------------------------------------------------
    const cv::FileNode camera = mapSection(root, "camera");
    readScalar<int>(path, camera, "width", &config.camera.width);
    readScalar<int>(path, camera, "height", &config.camera.height);
    readScalar<double>(path, camera, "fx", &config.camera.fx);
    readScalar<double>(path, camera, "fy", &config.camera.fy);
    readScalar<double>(path, camera, "cx", &config.camera.cx);
    readScalar<double>(path, camera, "cy", &config.camera.cy);
    readDoubleSequence(path, camera, "dist", &config.camera.dist);
    readScalar<std::string>(path, camera, "calibration_file",
                            &calibration_file);

    // ------------------------------------------------------------------
    // target
    // ------------------------------------------------------------------
    const cv::FileNode target = mapSection(root, "target");
    readScalar<double>(path, target, "disc_diameter_mm",
                       &config.target.disc_diameter_mm);
    readScalar<double>(path, target, "glow_outer_diameter_mm",
                       &config.target.glow_outer_diameter_mm);

    // ------------------------------------------------------------------
    // detector
    // ------------------------------------------------------------------
    const cv::FileNode detector = mapSection(root, "detector");
    readScalar<int>(path, detector, "green_h_min", &config.detector.green_h_min);
    readScalar<int>(path, detector, "green_h_max", &config.detector.green_h_max);
    readScalar<int>(path, detector, "green_s_min", &config.detector.green_s_min);
    readScalar<int>(path, detector, "green_v_min", &config.detector.green_v_min);
    readScalar<int>(path, detector, "morph_open_ksize",
                    &config.detector.morph_open_ksize);
    readScalar<int>(path, detector, "morph_close_ksize",
                    &config.detector.morph_close_ksize);
    readBool(path, detector, "roi_search_enabled",
             &config.detector.roi_search_enabled);
    readScalar<float>(path, detector, "roi_padding_scale",
                      &config.detector.roi_padding_scale);
    readScalar<int>(path, detector, "roi_min_size_px",
                    &config.detector.roi_min_size_px);
    readScalar<int>(path, detector, "min_area_px", &config.detector.min_area_px);
    readScalar<int>(path, detector, "max_area_px", &config.detector.max_area_px);
    readScalar<float>(path, detector, "min_axis_ratio",
                      &config.detector.min_axis_ratio);
    readScalar<float>(path, detector, "max_axis_ratio",
                      &config.detector.max_axis_ratio);
    readScalar<int>(path, detector, "white_v_min", &config.detector.white_v_min);
    readScalar<int>(path, detector, "white_s_max", &config.detector.white_s_max);
    readScalar<float>(path, detector, "ring_inner_scale",
                      &config.detector.ring_inner_scale);
    readScalar<float>(path, detector, "ring_outer_scale",
                      &config.detector.ring_outer_scale);
    readScalar<int>(path, detector, "ring_samples",
                    &config.detector.ring_samples);
    readScalar<float>(path, detector, "ring_green_min_ratio",
                      &config.detector.ring_green_min_ratio);
    readScalar<float>(path, detector, "score_threshold",
                      &config.detector.score_threshold);
    readBool(path, detector, "white_fallback_enabled",
             &config.detector.white_fallback_enabled);

    // ------------------------------------------------------------------
    // pose
    // ------------------------------------------------------------------
    const cv::FileNode pose = mapSection(root, "pose");
    readScalar<double>(path, pose, "min_distance_m", &config.pose.min_distance_m);
    readScalar<double>(path, pose, "max_distance_m", &config.pose.max_distance_m);
    readScalar<double>(path, pose, "max_reproj_rmse_px",
                       &config.pose.max_reproj_rmse_px);
    readScalar<int>(path, pose, "refine_iterations",
                    &config.pose.refine_iterations);
    readBool(path, pose, "enable_ambiguity_resolution",
             &config.pose.enable_ambiguity_resolution);
    readScalar<double>(path, pose, "ambiguity_cost_margin",
                       &config.pose.ambiguity_cost_margin);

    // ------------------------------------------------------------------
    // kalman
    // ------------------------------------------------------------------
    const cv::FileNode kalman = mapSection(root, "kalman");
    readScalar<double>(path, kalman, "process_noise_angle",
                       &config.kalman.process_noise_angle);
    readScalar<double>(path, kalman, "process_noise_rate",
                       &config.kalman.process_noise_rate);
    readScalar<double>(path, kalman, "meas_sigma_deg",
                       &config.kalman.meas_sigma_deg);
    readScalar<double>(path, kalman, "meas_sigma_scale_px",
                       &config.kalman.meas_sigma_scale_px);
    readScalar<double>(path, kalman, "gate_chi2", &config.kalman.gate_chi2);
    readScalar<int>(path, kalman, "max_prediction_frames",
                    &config.kalman.max_prediction_frames);
    readScalar<double>(path, kalman, "quality_decay_per_frame",
                       &config.kalman.quality_decay_per_frame);
    readScalar<double>(path, kalman, "range_process_noise",
                       &config.kalman.range_process_noise);
    readScalar<double>(path, kalman, "range_meas_sigma_m",
                       &config.kalman.range_meas_sigma_m);
    readScalar<double>(path, kalman, "latency_compensation_ms",
                       &config.kalman.latency_compensation_ms);

    // ------------------------------------------------------------------
    // tracker
    // ------------------------------------------------------------------
    const cv::FileNode tracker = mapSection(root, "tracker");
    readScalar<int>(path, tracker, "acquire_hits", &config.tracker.acquire_hits);
    readScalar<int>(path, tracker, "max_miss_frames",
                    &config.tracker.max_miss_frames);
    readScalar<int>(path, tracker, "full_search_interval",
                    &config.tracker.full_search_interval);
    readScalar<float>(path, tracker, "prediction_growth_per_frame",
                      &config.tracker.prediction_growth_per_frame);
    readScalar<int>(path, tracker, "roi_min_size_px",
                    &config.tracker.roi_min_size_px);

    // ------------------------------------------------------------------
    // output
    // ------------------------------------------------------------------
    const cv::FileNode output = mapSection(root, "output");
    readBool(path, output, "csv_enabled", &config.output.csv_enabled);
    readScalar<std::string>(path, output, "csv_path", &config.output.csv_path);
    readBool(path, output, "overlay_enabled", &config.output.overlay_enabled);
    readScalar<std::string>(path, output, "overlay_video_path",
                            &config.output.overlay_video_path);
    readBool(path, output, "serial_enabled", &config.output.serial_enabled);
    readScalar<std::string>(path, output, "serial_port",
                            &config.output.serial_port);
    readScalar<int>(path, output, "serial_baud", &config.output.serial_baud);

    // ------------------------------------------------------------------
    // root scalars
    // ------------------------------------------------------------------
    readScalar<std::string>(path, root, "source", &config.source_path);
    readScalar<double>(path, root, "nominal_fps", &config.nominal_fps);
  } catch (const std::runtime_error&) {
    throw;
  } catch (const cv::Exception& e) {
    throw std::runtime_error("config '" + path + "': " + e.what());
  }

  storage.release();

  // Camera calibration file, if requested, overrides the intrinsics above.
  applyCameraCalibrationFile(path, calibration_file, &config.camera);

  return config;
}

}  // namespace glowtrack
