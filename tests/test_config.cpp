// Tests for the pipeline/IO workstream:
//   * config loader (shipped YAML configs, defaults, missing file, calibration
//     file override),
//   * frame sources (image sequence, video file),
//   * output sinks (CSV header/schema, overlay renderer, overlay video writer),
//   * serial packet encoding + CRC (with a fake serial port).
#include "test_framework.hpp"

#include "glowtrack/config.hpp"
#include "glowtrack/frame_source.hpp"
#include "glowtrack/kalman_filter.hpp"
#include "glowtrack/output_sink.hpp"
#include "glowtrack/util.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace glowtrack;

namespace {

// OpenCV's FileStorage only accepts YAML documents that start with the
// %YAML:1.0 directive (the shipped config files do the same).
constexpr const char* kYamlHeader = "%YAML:1.0\n---\n";

int g_temp_counter = 0;

// Temporary directory that removes itself at scope exit. Prefers the system
// temp dir; falls back to a directory below the current working directory when
// the system temp dir is not writable (e.g. restricted CI sandboxes).
class TempDir {
 public:
  TempDir() {
    const std::string suffix =
        std::to_string(++g_temp_counter) + "_" +
        std::to_string(static_cast<unsigned long long>(
            std::llround(nowSeconds() * 1e6)));
    std::error_code code;
    std::filesystem::path candidate =
        std::filesystem::temp_directory_path(code) /
        ("glowtrack_pipeline_test_" + suffix);
    if (code) {
      candidate.clear();
    }
    if (!candidate.empty()) {
      code.clear();
      if (!std::filesystem::create_directories(candidate, code) || code) {
        candidate.clear();
      }
    }
    if (candidate.empty()) {
      candidate = std::filesystem::current_path() /
                  ("glowtrack_test_tmp_" + suffix);
      std::filesystem::create_directories(candidate);
    }
    path_ = candidate;
  }
  ~TempDir() {
    std::error_code code;
    std::filesystem::remove_all(path_, code);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;

  std::filesystem::path file(const std::string& name) const {
    return path_ / name;
  }
  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

// The unit tests are usually launched from the repository root, but ctest runs
// them from the build directory; try a few prefixes.
std::string findRepoFile(const std::string& relative_path) {
  const char* prefixes[] = {"", "../", "../../", "../../../"};
  for (const char* prefix : prefixes) {
    const std::string candidate = std::string(prefix) + relative_path;
    std::error_code code;
    if (std::filesystem::is_regular_file(candidate, code)) {
      return candidate;
    }
  }
  return std::string();
}

std::string findRepoDir(const std::string& relative_path) {
  const char* prefixes[] = {"", "../", "../../", "../../../"};
  for (const char* prefix : prefixes) {
    const std::string candidate = std::string(prefix) + relative_path;
    std::error_code code;
    if (std::filesystem::is_directory(candidate, code)) {
      return candidate;
    }
  }
  return std::string();
}

void writeTextFile(const std::filesystem::path& path,
                   const std::string& content) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  stream << content;
  stream.close();
}

// Writes a calibration YAML exactly like `glowtrack --mode calibrate`.
void writeCalibrationYaml(const std::filesystem::path& path, int width,
                          int height, double fx, double fy, double cx,
                          double cy) {
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy,
                           0.0, 0.0, 1.0);
  cv::Mat distortion = (cv::Mat_<double>(1, 5) << 0.01, -0.02, 0.001, 0.0, 0.0);
  cv::FileStorage storage(path.string(), cv::FileStorage::WRITE);
  storage << "image_width" << width;
  storage << "image_height" << height;
  storage << "camera_matrix" << camera_matrix;
  storage << "distortion_coefficients" << distortion;
  storage << "rms_reprojection_error" << 0.42;
  storage.release();
}

// Fake serial link capturing the bytes written by sendTrackPacket().
class FakeSerialPort final : public SerialPort {
 public:
  bool write(const uint8_t* data, size_t length) override {
    if (fail_write) {
      return false;
    }
    bytes.insert(bytes.end(), data, data + length);
    return true;
  }
  bool readExact(uint8_t*, size_t, int) override { return false; }
  int readSome(uint8_t*, size_t) override { return 0; }
  void close() override {}
  bool isOpen() const override { return true; }

  std::vector<uint8_t> bytes;
  bool fail_write = false;
};

uint16_t loadLe16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

}  // namespace

// ---------------------------------------------------------------------------
// Configuration loader
// ---------------------------------------------------------------------------
GT_TEST(config_loads_video_720p_config) {
  const std::string path = findRepoFile("config/video_720p_phone.yaml");
  GT_CHECK(!path.empty());
  const RuntimeConfig config = loadRuntimeConfig(path);

  GT_CHECK(config.camera.width == 720);
  GT_CHECK(config.camera.height == 1280);
  GT_CHECK_NEAR(config.camera.fx, 960.0, 1e-9);
  GT_CHECK_NEAR(config.camera.fy, 960.0, 1e-9);
  GT_CHECK_NEAR(config.camera.cx, 360.0, 1e-9);
  GT_CHECK_NEAR(config.camera.cy, 640.0, 1e-9);
  GT_CHECK(config.camera.dist.size() == 5);

  GT_CHECK_NEAR(config.target.disc_diameter_mm, 50.0, 1e-9);
  GT_CHECK_NEAR(config.target.glow_outer_diameter_mm, 62.0, 1e-9);

  GT_CHECK(config.detector.green_s_min == 50);
  GT_CHECK(config.detector.morph_close_ksize == 5);
  GT_CHECK(config.detector.roi_min_size_px == 80);
  GT_CHECK(config.detector.min_area_px == 200);
  GT_CHECK(config.detector.green_h_max == 100);
  GT_CHECK(config.detector.roi_search_enabled);
  GT_CHECK_NEAR(config.detector.roi_padding_scale, 2.5, 1e-6);

  GT_CHECK_NEAR(config.pose.max_reproj_rmse_px, 4.0, 1e-9);
  GT_CHECK(config.pose.refine_iterations == 20);

  GT_CHECK_NEAR(config.kalman.meas_sigma_deg, 0.35, 1e-9);
  GT_CHECK_NEAR(config.kalman.gate_chi2, 13.8, 1e-9);
  GT_CHECK(config.kalman.max_prediction_frames == 15);
  GT_CHECK_NEAR(config.kalman.latency_compensation_ms, 0.0, 1e-9);

  GT_CHECK(config.tracker.acquire_hits == 3);
  GT_CHECK(config.tracker.max_miss_frames == 15);
  GT_CHECK(config.tracker.full_search_interval == 5);
  GT_CHECK_NEAR(config.tracker.prediction_growth_per_frame, 1.15, 1e-6);
  GT_CHECK(config.tracker.roi_min_size_px == 80);

  GT_CHECK(config.output.csv_enabled);
  GT_CHECK(config.output.csv_path == "results/video_720p_track.csv");
  GT_CHECK(!config.output.overlay_enabled);
  GT_CHECK(!config.output.serial_enabled);

  GT_CHECK_NEAR(config.nominal_fps, 30.0, 1e-9);
}

GT_TEST(config_loads_openmv_config) {
  const std::string path = findRepoFile("config/openmv_h7plus_qvga.yaml");
  GT_CHECK(!path.empty());
  const RuntimeConfig config = loadRuntimeConfig(path);

  GT_CHECK(config.camera.width == 320);
  GT_CHECK(config.camera.height == 240);
  GT_CHECK_NEAR(config.camera.fx, 247.0, 1e-9);
  GT_CHECK_NEAR(config.camera.cx, 160.0, 1e-9);

  GT_CHECK(config.detector.green_s_min == 60);
  GT_CHECK(config.detector.morph_close_ksize == 3);
  GT_CHECK(config.detector.roi_min_size_px == 40);
  GT_CHECK(config.detector.min_area_px == 60);
  GT_CHECK(config.detector.ring_samples == 48);

  GT_CHECK_NEAR(config.pose.max_reproj_rmse_px, 3.0, 1e-9);
  GT_CHECK_NEAR(config.kalman.process_noise_angle, 0.0004, 1e-12);
  GT_CHECK_NEAR(config.kalman.meas_sigma_deg, 0.7, 1e-9);
  GT_CHECK(config.tracker.roi_min_size_px == 40);

  GT_CHECK(!config.output.serial_enabled);  // enabled at runtime via --serial
  GT_CHECK(config.output.serial_port == "COM3");
  GT_CHECK(config.output.serial_baud == 115200);
}

GT_TEST(config_loads_synthetic_config) {
  const std::string path = findRepoFile("config/synthetic_640x480.yaml");
  GT_CHECK(!path.empty());
  const RuntimeConfig config = loadRuntimeConfig(path);

  GT_CHECK(config.camera.width == 640);
  GT_CHECK(config.camera.height == 480);
  GT_CHECK_NEAR(config.camera.fx, 500.0, 1e-9);
  GT_CHECK_NEAR(config.camera.cx, 320.0, 1e-9);
  GT_CHECK_NEAR(config.camera.cy, 240.0, 1e-9);

  GT_CHECK(config.detector.min_area_px == 150);
  GT_CHECK(config.detector.max_area_px == 200000);
  GT_CHECK(config.detector.roi_min_size_px == 60);
  GT_CHECK(config.tracker.roi_min_size_px == 60);
  GT_CHECK(config.output.csv_path == "results/synthetic_track.csv");
}

GT_TEST(config_missing_keys_keep_defaults) {
  TempDir temp;
  const auto path = temp.file("minimal.yaml");
  writeTextFile(path,
                std::string(kYamlHeader) +
                    "camera:\n"
                    "  width: 111\n"
                    "  height: 222\n"
                    "  fx: 333.0\n"
                    "detector:\n"
                    "  min_area_px: 42\n");
  const RuntimeConfig config = loadRuntimeConfig(path.string());

  // Loaded values.
  GT_CHECK(config.camera.width == 111);
  GT_CHECK(config.camera.height == 222);
  GT_CHECK_NEAR(config.camera.fx, 333.0, 1e-9);
  GT_CHECK(config.detector.min_area_px == 42);

  // Defaults preserved everywhere else.
  const RuntimeConfig defaults;
  GT_CHECK_NEAR(config.camera.fy, defaults.camera.fy, 1e-9);
  GT_CHECK_NEAR(config.camera.cx, defaults.camera.cx, 1e-9);
  GT_CHECK(config.camera.dist.empty() == defaults.camera.dist.empty());
  GT_CHECK_NEAR(config.target.disc_diameter_mm,
                defaults.target.disc_diameter_mm, 1e-9);
  GT_CHECK(config.detector.green_h_min == defaults.detector.green_h_min);
  GT_CHECK(config.detector.green_s_min == defaults.detector.green_s_min);
  GT_CHECK(config.detector.morph_close_ksize ==
            defaults.detector.morph_close_ksize);
  GT_CHECK(config.detector.roi_search_enabled ==
            defaults.detector.roi_search_enabled);
  GT_CHECK_NEAR(config.pose.max_reproj_rmse_px,
                defaults.pose.max_reproj_rmse_px, 1e-12);
  GT_CHECK(config.kalman.max_prediction_frames ==
            defaults.kalman.max_prediction_frames);
  GT_CHECK_NEAR(config.kalman.quality_decay_per_frame,
                defaults.kalman.quality_decay_per_frame, 1e-12);
  GT_CHECK(config.tracker.acquire_hits == defaults.tracker.acquire_hits);
  GT_CHECK(config.tracker.max_miss_frames == defaults.tracker.max_miss_frames);
  GT_CHECK(config.output.csv_enabled == defaults.output.csv_enabled);
  GT_CHECK(config.output.overlay_enabled == defaults.output.overlay_enabled);
  GT_CHECK(config.output.serial_enabled == defaults.output.serial_enabled);
  GT_CHECK_NEAR(config.nominal_fps, defaults.nominal_fps, 1e-12);
  GT_CHECK(config.source_path.empty());
}

GT_TEST(config_missing_file_throws) {
  TempDir temp;
  const std::string missing = temp.file("does_not_exist.yaml").string();
  bool threw = false;
  try {
    (void)loadRuntimeConfig(missing);
  } catch (const std::runtime_error& error) {
    threw = true;
    const std::string message = error.what();
    GT_CHECK(message.find("config") != std::string::npos);
    GT_CHECK(message.find("does_not_exist.yaml") != std::string::npos);
  }
  GT_CHECK(threw);
}

GT_TEST(config_calibration_file_overrides_camera) {
  TempDir temp;
  writeCalibrationYaml(temp.file("calib.yaml"), 320, 240, 444.0, 445.0, 161.0,
                       121.0);
  const auto config_path = temp.file("track.yaml");
  writeTextFile(config_path,
                std::string(kYamlHeader) +
                    "camera:\n"
                    "  width: 640\n"
                    "  height: 480\n"
                    "  fx: 900.0\n"
                    "  fy: 900.0\n"
                    "  cx: 320.0\n"
                    "  cy: 240.0\n"
                    "  dist: [0.0, 0.0, 0.0, 0.0, 0.0]\n"
                    "  calibration_file: calib.yaml\n"
                    "tracker:\n"
                    "  acquire_hits: 5\n");

  const RuntimeConfig config = loadRuntimeConfig(config_path.string());
  // Calibration file (relative to the config file directory) wins.
  GT_CHECK(config.camera.width == 320);
  GT_CHECK(config.camera.height == 240);
  GT_CHECK_NEAR(config.camera.fx, 444.0, 1e-9);
  GT_CHECK_NEAR(config.camera.fy, 445.0, 1e-9);
  GT_CHECK_NEAR(config.camera.cx, 161.0, 1e-9);
  GT_CHECK_NEAR(config.camera.cy, 121.0, 1e-9);
  GT_CHECK(config.camera.dist.size() == 5);
  GT_CHECK_NEAR(config.camera.dist[0], 0.01, 1e-12);
  GT_CHECK(config.tracker.acquire_hits == 5);
}

GT_TEST(config_bad_calibration_file_throws) {
  TempDir temp;
  const auto config_path = temp.file("track.yaml");
  writeTextFile(config_path,
                std::string(kYamlHeader) +
                    "camera:\n"
                    "  calibration_file: missing_calib.yaml\n");
  bool threw = false;
  try {
    (void)loadRuntimeConfig(config_path.string());
  } catch (const std::runtime_error& error) {
    threw = true;
    GT_CHECK(std::string(error.what()).find("calibration_file") !=
             std::string::npos);
  }
  GT_CHECK(threw);
}

// ---------------------------------------------------------------------------
// Frame sources
// ---------------------------------------------------------------------------
GT_TEST(frame_source_image_sequence_natural_order) {
  TempDir temp;
  // Deliberately non-lexicographic names: natural sort must yield 0,1,2,10.
  const char* names[] = {"frame_10.png", "frame_2.png", "frame_0.png",
                         "frame_1.png"};
  const int values[] = {120, 80, 0, 40};  // grey value per file name order
  for (int i = 0; i < 4; ++i) {
    const cv::Mat image(32, 32, CV_8UC3,
                        cv::Scalar(values[i], values[i], values[i]));
    GT_CHECK(cv::imwrite((temp.file(names[i])).string(), image));
  }

  FrameSourceConfig source_config;
  source_config.type = FrameSourceConfig::Type::ImageSequence;
  source_config.path = temp.path().string();
  source_config.fps = 10.0;
  std::string error;
  const auto source = FrameSource::create(source_config, &error);
  GT_CHECK(source != nullptr);
  GT_CHECK(source->frameCount() == 4);
  GT_CHECK_NEAR(source->fps(), 10.0, 1e-9);
  GT_CHECK(!source->isLive());

  cv::Mat frame;
  double timestamp = -1.0;
  for (int i = 0; i < 4; ++i) {
    GT_CHECK(source->read(frame, timestamp));
    GT_CHECK(!frame.empty());
    GT_CHECK_NEAR(timestamp, static_cast<double>(i) / 10.0, 1e-9);
    GT_CHECK_NEAR(frame.at<cv::Vec3b>(0, 0)[0],
                  static_cast<unsigned char>(40 * i), 1e-6);
  }
  GT_CHECK(!source->read(frame, timestamp));
}

GT_TEST(frame_source_image_sequence_nested_fallback) {
  // Synthetic-validation layout: <scenario>/frames/frame_*.png with gt.csv and
  // meta.yaml next to the frames/ sub-directory. Passing --images <scenario>
  // must find the frames recursively and order them naturally.
  TempDir temp;
  std::filesystem::create_directories(temp.file("frames"));
  writeTextFile(temp.file("gt.csv"), "frame,timestamp_s\n0,0.0\n1,0.033\n2,0.066\n");
  writeTextFile(temp.file("meta.yaml"), "%YAML:1.0\n---\nscenario: dynamic\n");
  const char* names[] = {"frame_10.png", "frame_2.png", "frame_1.png"};
  const int values[] = {120, 80, 40};
  for (int i = 0; i < 3; ++i) {
    const cv::Mat image(24, 24, CV_8UC3,
                        cv::Scalar(values[i], values[i], values[i]));
    GT_CHECK(cv::imwrite((temp.file("frames") / names[i]).string(), image));
  }

  FrameSourceConfig source_config;
  source_config.type = FrameSourceConfig::Type::ImageSequence;
  source_config.path = temp.path().string();
  source_config.fps = 30.0;
  std::string error;
  const auto source = FrameSource::create(source_config, &error);
  GT_CHECK(source != nullptr);
  GT_CHECK(source->frameCount() == 3);

  cv::Mat frame;
  double timestamp = -1.0;
  const int expected[] = {40, 80, 120};  // frame_1, frame_2, frame_10
  for (int i = 0; i < 3; ++i) {
    GT_CHECK(source->read(frame, timestamp));
    GT_CHECK(!frame.empty());
    GT_CHECK_NEAR(timestamp, static_cast<double>(i) / 30.0, 1e-9);
    GT_CHECK_NEAR(frame.at<cv::Vec3b>(0, 0)[0],
                  static_cast<unsigned char>(expected[i]), 1e-6);
  }
  GT_CHECK(!source->read(frame, timestamp));
}

GT_TEST(frame_source_video_file) {
  const std::string path = findRepoFile("data/videos/v1_green_target.mp4");
  if (path.empty()) {
    std::printf("         [skip] validation video not found\n");
    return;
  }
  FrameSourceConfig source_config;
  source_config.type = FrameSourceConfig::Type::VideoFile;
  source_config.path = path;
  source_config.fps = 30.0;
  std::string error;
  const auto source = FrameSource::create(source_config, &error);
  GT_CHECK(source != nullptr);
  GT_CHECK(source->frameCount() > 400);
  GT_CHECK(source->fps() > 25.0);
  GT_CHECK(source->fps() < 35.0);
  GT_CHECK(!source->isLive());

  cv::Mat frame;
  double timestamp = -1.0;
  double previous = -1.0;
  for (int i = 0; i < 5; ++i) {
    GT_CHECK(source->read(frame, timestamp));
    GT_CHECK(!frame.empty());
    GT_CHECK(frame.cols == 720);
    GT_CHECK(frame.rows == 1280);
    GT_CHECK(timestamp > previous);
    previous = timestamp;
  }
}

GT_TEST(frame_source_missing_video_fails) {
  FrameSourceConfig source_config;
  source_config.type = FrameSourceConfig::Type::VideoFile;
  source_config.path = "this_video_does_not_exist.mp4";
  std::string error;
  const auto source = FrameSource::create(source_config, &error);
  GT_CHECK(source == nullptr);
  GT_CHECK(!error.empty());
}

// ---------------------------------------------------------------------------
// CSV / overlay outputs
// ---------------------------------------------------------------------------
GT_TEST(csv_logger_schema) {
  TempDir temp;
  const auto csv_path = temp.file("out/track.csv");
  CsvLogger logger(csv_path.string());
  GT_CHECK(logger.ok());
  logger.writeHeader();
  logger.writeHeader();  // idempotent

  TrackOutput output;
  output.frame_index = 3;
  output.timestamp_s = 0.1;
  output.state = TrackState::Tracking;
  output.valid = true;
  output.measured = true;
  output.yaw = 0.1;
  output.pitch = -0.05;
  output.yaw_rate = 1.0;
  output.pitch_rate = -0.5;
  output.distance_m = 0.75;
  output.quality = 0.9;
  output.track_id = 1;

  Detection detection;
  detection.valid = true;
  detection.disc_ellipse.center = cv::Point2f(300.f, 200.f);
  detection.disc_ellipse.semi_major = 20.f;
  detection.disc_ellipse.semi_minor = 18.f;
  detection.disc_ellipse.angle_deg = 10.f;
  PoseEstimate pose;
  pose.valid = true;
  pose.yaw = 0.09;
  pose.pitch = -0.04;
  pose.reproj_rmse_px = 0.3;
  logger.log(output, detection, pose);

  std::ifstream stream(csv_path);
  GT_CHECK(stream.good());
  std::string header;
  std::string row;
  std::getline(stream, header);
  std::getline(stream, row);
  GT_CHECK(header.find("frame,timestamp_s,state,detected,valid,measured,"
                       "predicted,yaw_deg,pitch_deg") == 0);
  GT_CHECK(header.find("ellipse_angle_deg") != std::string::npos);
  GT_CHECK(row.find("3,0.100000,TRACK,1,1,1,0,") == 0);
}

GT_TEST(overlay_renderer_draws) {
  RuntimeConfig config;
  config.camera.width = 640;
  config.camera.height = 480;
  config.camera.fx = 500.0;
  config.camera.fy = 500.0;
  config.camera.cx = 320.0;
  config.camera.cy = 240.0;

  OverlayRenderer renderer(config);
  const cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(10, 10, 10));

  Detection detection;
  detection.valid = true;
  detection.disc_ellipse.center = cv::Point2f(300.f, 200.f);
  detection.disc_ellipse.semi_major = 22.f;
  detection.disc_ellipse.semi_minor = 20.f;
  detection.glow_ellipse = detection.disc_ellipse;
  detection.glow_ellipse.semi_major = 32.f;
  detection.glow_ellipse.semi_minor = 30.f;

  PoseEstimate pose;
  pose.valid = true;
  pose.tvec = cv::Vec3d(10.0, -5.0, 700.0);

  TrackOutput output;
  output.state = TrackState::Tracking;
  output.valid = true;
  output.measured = true;
  output.yaw = 0.014;
  output.pitch = 0.007;
  output.distance_m = 0.7;
  output.quality = 0.9;
  output.frame_index = 7;
  output.track_id = 1;

  const cv::Rect roi(230, 130, 140, 140);
  const cv::Mat overlay =
      renderer.render(frame, output, detection, pose, roi, 3.5, 30.0);
  GT_CHECK(overlay.size() == frame.size());
  GT_CHECK(overlay.type() == frame.type());
  GT_CHECK(cv::norm(overlay, frame, cv::NORM_L1) > 0.0);
  // The input frame must not be modified.
  GT_CHECK_NEAR(cv::mean(frame)[0], 10.0, 1e-6);
}

GT_TEST(overlay_video_writer_mp4) {
  TempDir temp;
  const auto path = temp.file("overlay/out.mp4");
  {
    OverlayVideoWriter writer(path.string(), 30.0, 320, 240);
    GT_CHECK(writer.ok());
    for (int i = 0; i < 3; ++i) {
      const cv::Mat frame(240, 320, CV_8UC3,
                          cv::Scalar(static_cast<double>(i * 20), 40, 60));
      writer.write(frame);
    }
  }  // writer destructor flushes and closes the file
  std::error_code code;
  GT_CHECK(std::filesystem::is_regular_file(path, code));
  GT_CHECK(std::filesystem::file_size(path, code) > 0);
}

// ---------------------------------------------------------------------------
// Serial packet encoding
// ---------------------------------------------------------------------------
GT_TEST(serial_packet_encoding) {
  FakeSerialPort port;
  TrackOutput output;
  output.valid = true;
  output.measured = true;
  output.predicted = false;
  output.state = TrackState::Tracking;
  output.yaw = 0.1234;
  output.pitch = -0.5;
  output.yaw_rate = 2.0;
  output.pitch_rate = -3.0;
  output.distance_m = 1.5;
  output.quality = 0.87;

  std::string error;
  GT_CHECK(sendTrackPacket(port, output, 7, &error));
  GT_CHECK(port.bytes.size() == sizeof(TrackPacket));
  GT_CHECK(port.bytes.size() == 19);

  const uint8_t* bytes = port.bytes.data();
  GT_CHECK(bytes[0] == 0xAA);
  GT_CHECK(bytes[1] == 0x55);
  GT_CHECK(bytes[2] == 1);   // version
  GT_CHECK(bytes[3] == 7);   // seq
  GT_CHECK(bytes[4] == 0x13);  // valid | measured | tracking
  GT_CHECK_NEAR(static_cast<double>(static_cast<int16_t>(loadLe16(bytes + 5))),
                1234.0, 0.0);  // yaw 0.1234 rad in 1e-4
  GT_CHECK_NEAR(static_cast<double>(static_cast<int16_t>(loadLe16(bytes + 7))),
                -5000.0, 0.0);  // pitch -0.5 rad
  GT_CHECK_NEAR(static_cast<double>(static_cast<int16_t>(loadLe16(bytes + 9))),
                2000.0, 0.0);  // yaw rate 2 rad/s in 1e-3
  GT_CHECK_NEAR(static_cast<double>(static_cast<int16_t>(loadLe16(bytes + 11))),
                -3000.0, 0.0);
  GT_CHECK(loadLe16(bytes + 13) == 1500);  // distance 1.5 m -> mm
  GT_CHECK(bytes[15] == 87);               // quality 0.87 -> 87
  GT_CHECK(bytes[16] == 0);                // reserved
  const uint16_t crc = loadLe16(bytes + 17);
  GT_CHECK(crc == crc16Ccitt(bytes, 17));
}

GT_TEST(serial_packet_saturation_and_flags) {
  FakeSerialPort port;
  TrackOutput output;
  output.valid = false;
  output.measured = false;
  output.predicted = true;
  output.state = TrackState::Acquiring;
  output.distance_m = 200.0;  // saturate uint16
  output.quality = 1.5;       // clamp to 100

  std::string error;
  GT_CHECK(sendTrackPacket(port, output, 0, &error));
  const uint8_t* bytes = port.bytes.data();
  GT_CHECK(bytes[4] == 0x0C);  // predicted | acquiring
  GT_CHECK(loadLe16(bytes + 13) == 65535);
  GT_CHECK(bytes[15] == 100);

  TrackOutput huge;
  huge.yaw = 5.0;
  huge.pitch = -5.0;
  huge.yaw_rate = 60.0;
  huge.pitch_rate = -60.0;
  FakeSerialPort port2;
  GT_CHECK(sendTrackPacket(port2, huge, 1, &error));
  const uint8_t* bytes2 = port2.bytes.data();
  GT_CHECK(static_cast<int16_t>(loadLe16(bytes2 + 5)) == 32767);    // yaw_q
  GT_CHECK(static_cast<int16_t>(loadLe16(bytes2 + 7)) == -32768);   // pitch_q
  GT_CHECK(static_cast<int16_t>(loadLe16(bytes2 + 9)) == 32767);    // yaw_rate_q
  GT_CHECK(static_cast<int16_t>(loadLe16(bytes2 + 11)) == -32768);  // pitch_rate_q
}

GT_TEST(serial_packet_write_failure) {
  FakeSerialPort port;
  port.fail_write = true;
  TrackOutput output;
  output.state = TrackState::Tracking;
  std::string error;
  GT_CHECK(!sendTrackPacket(port, output, 0, &error));
  GT_CHECK(!error.empty());
}

GT_TEST(serial_port_open_missing_port_fails) {
  std::string error;
  const auto serial = SerialPort::open("COM_does_not_exist_999", 115200, &error);
  GT_CHECK(serial == nullptr);
  GT_CHECK(!error.empty());
}
