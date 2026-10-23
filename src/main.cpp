// Command line entry point of the green glow tracker.
//
//   Track mode (default) -- run the full vision pipeline on a video, an image
//   sequence or a live camera and write CSV / overlay / serial outputs.
//
//   Calibrate mode -- estimate camera intrinsics from chessboard images.

#include "glowtrack/glowtrack.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace {

using namespace glowtrack;

constexpr const char* kVersion = "0.1.0";

void printUsage() {
  std::printf(
      "green_glow_tracker %s -- upper-computer vision module for a green glowing\n"
      "target (RM unpowered glider project). Detects the green glow, solves the\n"
      "target pose with PnP, filters the line-of-sight angles and streams yaw/pitch\n"
      "to the lower computer.\n\n"
      "Usage:\n"
      "  glowtrack --config <yaml> --video  <file>  [options]\n"
      "  glowtrack --config <yaml> --images <dir>   [--fps 30] [options]\n"
      "  glowtrack --config <yaml> --camera <index> [options]\n"
      "  glowtrack --config <yaml> --openmv <COMx>  [--baud 921600] [options]\n"
      "  glowtrack --mode calibrate --images <dir> --cols 9 --rows 6 \\\n"
      "            --square-mm 25 --out calib.yaml [--debug-dir <dir>]\n\n"
      "Tracking options:\n"
      "  --start-frame N   first frame index to process (default 0)\n"
      "  --max-frames  N   stop after N frames (default: all)\n"
      "  --csv <path>      write per-frame CSV log      (overrides config)\n"
      "  --no-csv          disable CSV logging\n"
      "  --overlay <path>  write annotated mp4 video    (overrides config)\n"
      "  --serial <port>   stream TrackPacket to the lower computer\n"
      "  --baud <rate>     serial baud rate (default from config)\n"
      "  --latency-ms <ms> control latency compensation (overrides config)\n"
      "  --print-every N   progress line every N frames (default 30, 0 = off)\n"
      "  --quiet           only print the final summary\n"
      "  --no-csv          disable the CSV log\n\n"
      "Calibration options:\n"
      "  --cols / --rows   inner chessboard corners (default 9 x 6)\n"
      "  --square-mm       chessboard square size in mm (default 25)\n"
      "  --out <yaml>      output calibration file\n"
      "  --debug-dir <dir> save annotated corner detections to this directory\n\n"
      "Other:\n"
      "  --help / -h       show this help\n"
      "  --version         print version\n",
      kVersion);
}

struct Args {
  std::string config = "config/video_720p_phone.yaml";
  std::string mode = "track";
  std::string video;
  std::string images;
  std::string camera;
  std::string openmv;
  std::string csv;
  std::string overlay;
  std::string serial;
  std::string out;
  std::string debug_dir;
  int baud = 0;
  int cols = 9;
  int rows = 6;
  double square_mm = 25.0;
  int start_frame = 0;
  int max_frames = -1;
  int print_every = 30;
  double latency_ms = -1.0;
  double fps = -1.0;
  bool no_csv = false;
  bool no_overlay = false;
  bool quiet = false;
  bool help = false;
  bool version = false;
};

bool takeValue(int argc, char** argv, int* i, const char* name,
               std::string* value) {
  if (*i + 1 >= argc) {
    std::fprintf(stderr, "error: missing value for %s\n", name);
    return false;
  }
  *value = argv[++(*i)];
  return true;
}

bool parseArgs(int argc, char** argv, Args* args) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto needStr = [&](std::string* dst) {
      return takeValue(argc, argv, &i, arg.c_str(), dst);
    };
    if (arg == "--help" || arg == "-h") {
      args->help = true;
    } else if (arg == "--version") {
      args->version = true;
    } else if (arg == "--config") {
      if (!needStr(&args->config)) return false;
    } else if (arg == "--mode") {
      if (!needStr(&args->mode)) return false;
    } else if (arg == "--video") {
      if (!needStr(&args->video)) return false;
    } else if (arg == "--images") {
      if (!needStr(&args->images)) return false;
    } else if (arg == "--camera") {
      if (!needStr(&args->camera)) return false;
    } else if (arg == "--openmv") {
      if (!needStr(&args->openmv)) return false;
    } else if (arg == "--csv") {
      if (!needStr(&args->csv)) return false;
    } else if (arg == "--overlay") {
      if (!needStr(&args->overlay)) return false;
    } else if (arg == "--serial") {
      if (!needStr(&args->serial)) return false;
    } else if (arg == "--out") {
      if (!needStr(&args->out)) return false;
    } else if (arg == "--debug-dir") {
      if (!needStr(&args->debug_dir)) return false;
    } else if (arg == "--no-csv") {
      args->no_csv = true;
    } else if (arg == "--no-overlay") {
      args->no_overlay = true;
    } else if (arg == "--quiet") {
      args->quiet = true;
    } else if (arg == "--baud") {
      std::string v;
      if (!needStr(&v)) return false;
      args->baud = std::atoi(v.c_str());
    } else if (arg == "--cols") {
      std::string v;
      if (!needStr(&v)) return false;
      args->cols = std::atoi(v.c_str());
    } else if (arg == "--rows") {
      std::string v;
      if (!needStr(&v)) return false;
      args->rows = std::atoi(v.c_str());
    } else if (arg == "--square-mm") {
      std::string v;
      if (!needStr(&v)) return false;
      args->square_mm = std::atof(v.c_str());
    } else if (arg == "--start-frame") {
      std::string v;
      if (!needStr(&v)) return false;
      args->start_frame = std::atoi(v.c_str());
    } else if (arg == "--max-frames") {
      std::string v;
      if (!needStr(&v)) return false;
      args->max_frames = std::atoi(v.c_str());
    } else if (arg == "--print-every") {
      std::string v;
      if (!needStr(&v)) return false;
      args->print_every = std::atoi(v.c_str());
    } else if (arg == "--latency-ms") {
      std::string v;
      if (!needStr(&v)) return false;
      args->latency_ms = std::atof(v.c_str());
    } else if (arg == "--fps") {
      std::string v;
      if (!needStr(&v)) return false;
      args->fps = std::atof(v.c_str());
    } else {
      std::fprintf(stderr, "error: unknown argument '%s' (try --help)\n",
                   arg.c_str());
      return false;
    }
  }
  return true;
}

FrameSourceConfig makeSourceConfig(const Args& args) {
  FrameSourceConfig source;
  if (!args.video.empty()) {
    source.type = FrameSourceConfig::Type::VideoFile;
    source.path = args.video;
  } else if (!args.images.empty()) {
    source.type = FrameSourceConfig::Type::ImageSequence;
    source.path = args.images;
  } else if (!args.camera.empty()) {
    source.type = FrameSourceConfig::Type::Camera;
    source.path = args.camera;
  } else {
    source.type = FrameSourceConfig::Type::OpenMvSerial;
    source.path = args.openmv;
  }
  return source;
}

// ---------------------------------------------------------------------------
// Calibration mode
// ---------------------------------------------------------------------------
int runCalibration(const Args& args) {
  if (args.images.empty()) {
    std::fprintf(stderr, "error: --mode calibrate needs --images <dir>\n");
    return 1;
  }
  if (args.out.empty()) {
    std::fprintf(stderr,
                 "error: --mode calibrate needs --out <calibration.yaml>\n");
    return 1;
  }

  std::vector<std::string> files;
  {
    std::error_code ec;
    for (const auto& entry :
         std::filesystem::directory_iterator(args.images, ec)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const auto ext = entry.path().extension().string();
      std::string low;
      std::transform(ext.begin(), ext.end(), std::back_inserter(low),
                     [](unsigned char c) { return std::tolower(c); });
      if (low == ".png" || low == ".jpg" || low == ".jpeg" || low == ".bmp" ||
          low == ".tif" || low == ".tiff") {
        files.push_back(entry.path().string());
      }
    }
    std::sort(files.begin(), files.end());
  }
  if (files.empty()) {
    std::fprintf(stderr, "error: no images found in %s\n", args.images.c_str());
    return 1;
  }

  const cv::Size pattern(args.cols, args.rows);
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<cv::Point3f> single_object;
  for (int r = 0; r < args.rows; ++r) {
    for (int c = 0; c < args.cols; ++c) {
      single_object.emplace_back(static_cast<float>(c * args.square_mm),
                                 static_cast<float>(r * args.square_mm), 0.f);
    }
  }

  for (const auto& file : files) {
    const cv::Mat gray = cv::imread(file, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      continue;
    }
    std::vector<cv::Point2f> corners;
    const int flags =
        cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE;
    bool found = false;
    if (cv::findChessboardCornersSB(gray, pattern, corners, flags)) {
      found = true;
    } else if (cv::findChessboardCorners(gray, pattern, corners, flags)) {
      cv::cornerSubPix(
          gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                           0.01));
      found = true;
    }
    if (!found) {
      continue;
    }
    object_points.push_back(single_object);
    image_points.push_back(corners);
    if (!args.debug_dir.empty()) {
      cv::Mat color;
      cv::cvtColor(gray, color, cv::COLOR_GRAY2BGR);
      cv::drawChessboardCorners(color, pattern, corners, true);
      cv::imwrite((std::filesystem::path(args.debug_dir) /
                   std::filesystem::path(file).filename())
                      .string(),
                  color);
    }
  }

  if (image_points.size() < 5) {
    std::fprintf(stderr,
                 "error: only %zu valid chessboard views (need >= 5)\n",
                 image_points.size());
    return 1;
  }

  const cv::Size image_size = cv::imread(files.front()).size();
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
  std::vector<cv::Mat> rvecs, tvecs;
  const double rms = cv::calibrateCamera(object_points, image_points, image_size,
                                         camera_matrix, dist, rvecs, tvecs);
  std::printf("Calibration: %zu views, %dx%d, RMS reprojection = %.4f px\n",
              image_points.size(), image_size.width, image_size.height, rms);

  ensureParentDirectory(args.out);
  cv::FileStorage fs(args.out, cv::FileStorage::WRITE);
  fs << "image_width" << image_size.width;
  fs << "image_height" << image_size.height;
  fs << "camera_matrix" << camera_matrix;
  fs << "distortion_coefficients" << dist;
  fs << "rms_reprojection_error" << rms;
  fs.release();
  std::printf("Wrote %s\n", args.out.c_str());
  return 0;
}

// ---------------------------------------------------------------------------
// Track mode
// ---------------------------------------------------------------------------
int runTracking(const Args& args) {
  RuntimeConfig config;
  try {
    config = loadRuntimeConfig(args.config);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: cannot load config '%s': %s\n",
                 args.config.c_str(), e.what());
    return 1;
  }
  if (args.fps > 0.0) {
    config.nominal_fps = args.fps;
  }
  if (args.latency_ms >= 0.0) {
    config.kalman.latency_compensation_ms = args.latency_ms;
  }
  if (args.no_csv) {
    config.output.csv_enabled = false;
  }
  if (!args.csv.empty()) {
    config.output.csv_enabled = true;
    config.output.csv_path = args.csv;
  }
  if (args.no_overlay) {
    config.output.overlay_enabled = false;
  }
  if (!args.overlay.empty()) {
    config.output.overlay_enabled = true;
    config.output.overlay_video_path = args.overlay;
  }
  if (!args.serial.empty()) {
    config.output.serial_enabled = true;
    config.output.serial_port = args.serial;
  }
  if (args.baud > 0) {
    config.output.serial_baud = args.baud;
  }

  FrameSourceConfig source_config = makeSourceConfig(args);
  source_config.fps = args.fps > 0.0 ? args.fps : config.nominal_fps;
  source_config.serial_baud = config.output.serial_baud;

  std::string error;
  auto source = FrameSource::create(source_config, &error);
  if (!source) {
    std::fprintf(stderr, "error: cannot open frame source: %s\n", error.c_str());
    return 1;
  }

  TargetTracker tracker(config);

  std::unique_ptr<CsvLogger> csv;
  if (config.output.csv_enabled) {
    csv = std::make_unique<CsvLogger>(config.output.csv_path);
    if (csv->ok()) {
      csv->writeHeader();
    } else {
      std::fprintf(stderr, "warning: CSV log disabled: %s\n",
                   csv->lastError().c_str());
    }
  }

  std::unique_ptr<OverlayRenderer> renderer;
  std::unique_ptr<OverlayVideoWriter> video;
  if (config.output.overlay_enabled) {
    renderer = std::make_unique<OverlayRenderer>(config);
  }

  std::unique_ptr<SerialPort> serial;
  if (config.output.serial_enabled) {
    serial = SerialPort::open(config.output.serial_port,
                              config.output.serial_baud, &error);
    if (!serial) {
      std::fprintf(stderr, "warning: serial disabled: %s\n", error.c_str());
    }
  }

  const double source_fps = source->fps() > 0.0 ? source->fps() : 30.0;

  int frame_index = 0;
  int processed = 0;
  int measured = 0;
  int valid = 0;
  uint8_t serial_seq = 0;
  double proc_ms_sum = 0.0;
  double wall_start = nowSeconds();
  double last_timestamp = 0.0;

  cv::Mat frame;
  double timestamp = 0.0;
  while (source->read(frame, timestamp)) {
    if (frame_index < args.start_frame) {
      ++frame_index;
      continue;
    }
    if (frame.empty()) {
      ++frame_index;
      continue;
    }
    if (!video && renderer && config.output.overlay_enabled &&
        frame.cols > 0) {
      video = std::make_unique<OverlayVideoWriter>(
          config.output.overlay_video_path, source_fps, frame.cols, frame.rows);
      if (!video->ok()) {
        std::fprintf(stderr, "warning: overlay disabled: %s\n",
                     video->lastError().c_str());
        video.reset();
        renderer.reset();
      }
    }

    ScopedTimer timer;
    const TrackOutput output = tracker.update(frame, timestamp, frame_index);
    const double proc_ms = timer.elapsedMs();
    proc_ms_sum += proc_ms;
    last_timestamp = timestamp;
    ++processed;
    if (output.measured) ++measured;
    if (output.valid) ++valid;

    if (csv && csv->ok()) {
      csv->log(output, tracker.lastDetection(), tracker.lastPose());
    }
    if (serial) {
      std::string send_error;
      if (!sendTrackPacket(*serial, output, serial_seq++, &send_error)) {
        std::fprintf(stderr, "warning: serial send failed: %s\n",
                     send_error.c_str());
        serial.reset();
      }
    }
    if (renderer && video) {
      const cv::Mat overlay =
          renderer->render(frame, output, tracker.lastDetection(),
                           tracker.lastPose(), tracker.currentSearchRoi(),
                           proc_ms, source_fps);
      video->write(overlay);
    }

    if (!args.quiet && args.print_every > 0 &&
        processed % args.print_every == 0) {
      std::printf(
          "frame %5d | %-7s | yaw %+7.2f deg | pitch %+6.2f deg | range "
          "%5.2f m | q %.2f | %.1f ms\n",
          frame_index, tracker.stateName().c_str(),
          output.yaw * kRad2Deg, output.pitch * kRad2Deg, output.distance_m,
          output.quality, proc_ms);
      std::fflush(stdout);
    }

    ++frame_index;
    if (args.max_frames > 0 && processed >= args.max_frames) {
      break;
    }
  }

  if (video) {
    video.reset();  // flush
  }

  const double wall = nowSeconds() - wall_start;
  const double fps = processed > 0 ? processed / std::max(wall, 1e-6) : 0.0;
  std::printf("\n===== summary =====\n");
  std::printf("frames processed : %d (last timestamp %.3f s)\n", processed,
              last_timestamp);
  std::printf("detected frames  : %d (%.1f %%)\n", measured,
              100.0 * measured / std::max(processed, 1));
  std::printf("valid output     : %d (%.1f %%)\n", valid,
              100.0 * valid / std::max(processed, 1));
  std::printf("mean process time: %.2f ms/frame\n",
              proc_ms_sum / std::max(processed, 1));
  std::printf("throughput       : %.1f fps\n", fps);
  if (csv && csv->ok()) {
    std::printf("csv log          : %s\n", csv->path().c_str());
  }
  if (config.output.overlay_enabled && !config.output.overlay_video_path.empty()) {
    std::printf("overlay video    : %s\n",
                config.output.overlay_video_path.c_str());
  }
  if (serial) {
    std::printf("serial stream    : running (port closed on exit)\n");
  }
  return processed > 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parseArgs(argc, argv, &args)) {
    return 1;
  }
  if (args.help) {
    printUsage();
    return 0;
  }
  if (args.version) {
    std::printf("green_glow_tracker %s (OpenCV %s)\n", kVersion, CV_VERSION);
    return 0;
  }
  if (args.mode == "calibrate") {
    return runCalibration(args);
  }
  if (args.mode != "track") {
    std::fprintf(stderr, "error: unknown --mode '%s'\n", args.mode.c_str());
    return 1;
  }
  if (args.video.empty() && args.images.empty() && args.camera.empty() &&
      args.openmv.empty()) {
    std::fprintf(stderr,
                 "error: one of --video / --images / --camera / --openmv is "
                 "required\n");
    return 1;
  }
  return runTracking(args);
}
