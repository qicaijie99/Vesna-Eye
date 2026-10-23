// Developer helper for the detector workstream.
//
//   Frame dump mode (default) -- detect selected frames, write debug images and
//   print the ellipse / candidate details:
//     glowtrack_devdet --video <file> --config <yaml> --frames 0,120,300 --out dir
//
//   Whole-video mode -- recall, timing, optional reference check, CSV log:
//     glowtrack_devdet --video <file> --config <yaml> --all [--check]
//                       [--roi] [--csv results/det.csv] [--dump-every 60]
//
// The detector config is read directly from the YAML file (cv::FileStorage) so
// this tool does not depend on the shared config loader workstream.
//
// Owned by the detector workstream -- extend freely.

#include "glowtrack/config.hpp"
#include "glowtrack/detector.hpp"
#include "glowtrack/util.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// Minimal YAML reader for the detector section.
// ---------------------------------------------------------------------------
template <typename T>
bool readScalar(const cv::FileNode& parent, const char* key, T* value) {
  const cv::FileNode node = parent[key];
  if (node.empty()) {
    return false;
  }
  try {
    node >> *value;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool readBool(const cv::FileNode& parent, const char* key, bool* value) {
  int as_int = *value ? 1 : 0;
  if (!readScalar(parent, key, &as_int)) {
    return false;
  }
  *value = as_int != 0;
  return true;
}

glowtrack::DetectionConfig loadDetectorConfig(const std::string& path) {
  glowtrack::DetectionConfig config;  // defaults from config.hpp
  try {
    // OpenCV's YAML reader requires the "%YAML:1.0" directive, which the
    // hand-written config files do not carry; read the text, prepend the
    // directive and parse it from memory.
    std::ifstream input(path, std::ios::binary);
    if (!input.good()) {
      std::fprintf(stderr, "warning: cannot open config %s, using defaults\n",
                   path.c_str());
      return config;
    }
    std::string text((std::istreambuf_iterator<char>(input)),
                     std::istreambuf_iterator<char>());
    if (text.rfind("%YAML", 0) != 0) {
      text = "%YAML:1.0\n" + text;
    }
    cv::FileStorage fs(text, cv::FileStorage::READ |
                                 cv::FileStorage::MEMORY |
                                 cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      std::fprintf(stderr, "warning: cannot parse config %s, using defaults\n",
                   path.c_str());
      return config;
    }
    const cv::FileNode node = fs["detector"];
    if (node.empty()) {
      std::fprintf(stderr,
                   "warning: no 'detector' section in %s, using defaults\n",
                   path.c_str());
      return config;
    }
    readScalar(node, "green_h_min", &config.green_h_min);
    readScalar(node, "green_h_max", &config.green_h_max);
    readScalar(node, "green_s_min", &config.green_s_min);
    readScalar(node, "green_v_min", &config.green_v_min);
    readScalar(node, "morph_open_ksize", &config.morph_open_ksize);
    readScalar(node, "morph_close_ksize", &config.morph_close_ksize);
    readBool(node, "roi_search_enabled", &config.roi_search_enabled);
    readScalar(node, "roi_padding_scale", &config.roi_padding_scale);
    readScalar(node, "roi_min_size_px", &config.roi_min_size_px);
    readScalar(node, "min_area_px", &config.min_area_px);
    readScalar(node, "max_area_px", &config.max_area_px);
    readScalar(node, "min_axis_ratio", &config.min_axis_ratio);
    readScalar(node, "max_axis_ratio", &config.max_axis_ratio);
    readScalar(node, "white_v_min", &config.white_v_min);
    readScalar(node, "white_s_max", &config.white_s_max);
    readScalar(node, "ring_inner_scale", &config.ring_inner_scale);
    readScalar(node, "ring_outer_scale", &config.ring_outer_scale);
    readScalar(node, "ring_samples", &config.ring_samples);
    readScalar(node, "ring_green_min_ratio", &config.ring_green_min_ratio);
    readScalar(node, "score_threshold", &config.score_threshold);
    readBool(node, "white_fallback_enabled", &config.white_fallback_enabled);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "warning: config read failed (%s), using defaults\n",
                 e.what());
  }
  return config;
}

std::vector<int> parseFrameList(const std::string& text) {
  std::vector<int> frames;
  size_t start = 0;
  while (start <= text.size()) {
    const size_t comma = text.find(',', start);
    const std::string token =
        text.substr(start, comma == std::string::npos ? std::string::npos
                                                      : comma - start);
    if (!token.empty()) {
      frames.push_back(std::atoi(token.c_str()));
    }
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  return frames;
}

bool containsFrame(const std::vector<int>& frames, int index) {
  for (int f : frames) {
    if (f == index) return true;
  }
  return false;
}

// ---------------------------------------------------------------------------
// Independent reference disc (for recall / false-positive numbers).
//
// Uses the recon report's strict overexposed-white mask (V >= 225, S <= 45),
// picks the round blob with the best green annulus coverage. It is deliberately
// simpler than the detector and shares no fitting code with it.
// ---------------------------------------------------------------------------
struct ReferenceDisc {
  bool valid = false;
  cv::Point2f center{0.f, 0.f};
  float semi_major = 0.f;
  float semi_minor = 0.f;
  float angle_deg = 0.f;
  float ring_ratio = 0.f;
  float area = 0.f;
};

float annulusGreenRatio(const cv::Mat& green, const ReferenceDisc& disc) {
  const float ca = std::cos(disc.angle_deg * static_cast<float>(CV_PI) / 180.f);
  const float sa = std::sin(disc.angle_deg * static_cast<float>(CV_PI) / 180.f);
  const int samples = 48;
  float green_hits = 0.f;
  float total = 0.f;
  for (int ri = 0; ri < 3; ++ri) {
    const float scale = 1.02f + 0.215f * static_cast<float>(ri);
    for (int si = 0; si < samples; ++si) {
      const float angle = 2.f * static_cast<float>(CV_PI) * si / samples;
      const float u = scale * std::cos(angle);
      const float v = scale * std::sin(angle);
      const int x = cvRound(disc.center.x + disc.semi_major * u * ca -
                            disc.semi_minor * v * sa);
      const int y = cvRound(disc.center.y + disc.semi_major * u * sa +
                            disc.semi_minor * v * ca);
      if (x < 0 || y < 0 || x >= green.cols || y >= green.rows) continue;
      total += 1.f;
      if (green.at<unsigned char>(y, x) != 0) green_hits += 1.f;
    }
  }
  return total > 0.f ? green_hits / total : 0.f;
}

ReferenceDisc findReferenceDisc(const cv::Mat& bgr,
                                const glowtrack::DetectionConfig& config) {
  ReferenceDisc best;
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::Mat green;
  cv::inRange(hsv,
              cv::Scalar(config.green_h_min, config.green_s_min,
                         config.green_v_min),
              cv::Scalar(config.green_h_max, 255, 255), green);
  auto kernel = [](int ksize) {
    const int odd = (ksize % 2 == 0) ? ksize + 1 : ksize;
    return odd > 1 ? cv::getStructuringElement(cv::MORPH_RECT,
                                               cv::Size(odd, odd))
                   : cv::Mat();
  };
  const cv::Mat open_kernel = kernel(config.morph_open_ksize);
  const cv::Mat close_kernel = kernel(config.morph_close_ksize);
  if (!open_kernel.empty()) {
    cv::morphologyEx(green, green, cv::MORPH_OPEN, open_kernel);
  }
  if (!close_kernel.empty()) {
    cv::morphologyEx(green, green, cv::MORPH_CLOSE, close_kernel);
  }

  cv::Mat white;
  cv::inRange(hsv, cv::Scalar(0, 0, 225), cv::Scalar(180, 45, 255), white);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(white, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  for (const auto& contour : contours) {
    if (contour.size() < 5) continue;
    const float area = static_cast<float>(cv::contourArea(contour));
    if (area < 300.f || area > 70000.f) continue;
    const cv::RotatedRect rect = cv::fitEllipse(contour);
    const float major = std::max(rect.size.width, rect.size.height);
    const float minor = std::min(rect.size.width, rect.size.height);
    if (major <= 0.f || minor / major < 0.5f) continue;
    ReferenceDisc disc;
    disc.valid = true;
    disc.center = rect.center;
    disc.semi_major = 0.5f * major;
    disc.semi_minor = 0.5f * minor;
    disc.angle_deg = rect.angle;
    disc.area = area;
    disc.ring_ratio = annulusGreenRatio(green, disc);
    if (!best.valid || disc.ring_ratio > best.ring_ratio) {
      best = disc;
    }
  }
  return best;
}

struct Options {
  std::string video;
  std::string config_path = "config/video_720p_phone.yaml";
  std::string out_dir = "results/devdet";
  std::string csv_path;
  std::string frames_text = "0";
  int max_frames = -1;
  int dump_every = 0;
  bool all_frames = false;
  bool check = false;
  bool roi = false;
  bool compare_roi = false;
  bool no_debug = false;
  bool verbose = false;
};

void dumpDebugImages(const glowtrack::TargetDetector& detector,
                     const cv::Mat& frame, int index,
                     const std::string& out_dir) {
  std::filesystem::create_directories(out_dir);
  char name[160];
  const std::filesystem::path dir(out_dir);
  std::snprintf(name, sizeof(name), "frame_%05d.png", index);
  cv::imwrite((dir / name).string(), frame);
  if (!detector.debugMask().empty()) {
    std::snprintf(name, sizeof(name), "mask_%05d.png", index);
    cv::imwrite((dir / name).string(), detector.debugMask());
  }
  if (!detector.debugCandidates().empty()) {
    std::snprintf(name, sizeof(name), "cand_%05d.png", index);
    cv::imwrite((dir / name).string(), detector.debugCandidates());
  }
}

void printCandidateInfo(const glowtrack::TargetDetector& detector) {
  int index = 0;
  for (const auto& candidate : detector.debugCandidateInfo()) {
    std::printf(
        "    cand[%d] %-4s c=(%.1f,%.1f) axes=(%.1f,%.1f) ang=%.1f "
        "score=%.3f fill=%.2f ring=%.2f wf=%.2f core=%.2f V=%.0f S=%.0f "
        "glow=%.0f\n",
        index++, candidate.from_hole ? "hole" : "whit", candidate.disc.center.x,
        candidate.disc.center.y, candidate.disc.semi_major,
        candidate.disc.semi_minor, candidate.disc.angle_deg, candidate.score,
        candidate.fill_ratio, candidate.ring_ratio, candidate.white_frac,
        candidate.core_frac, candidate.mean_v, candidate.mean_s,
        candidate.glow_area);
  }
}

// Overlay used for the "all frames" mode dumps.
cv::Mat renderOverlay(const cv::Mat& frame,
                      const glowtrack::Detection& detection,
                      const ReferenceDisc& reference, bool roi_mode,
                      const cv::Rect& roi) {
  cv::Mat overlay = frame.clone();
  if (roi.width > 0 && roi_mode) {
    cv::rectangle(overlay, roi, cv::Scalar(255, 90, 0), 1);
  }
  if (reference.valid) {
    cv::ellipse(overlay,
                cv::Point(cvRound(reference.center.x),
                          cvRound(reference.center.y)),
                cv::Size2f(reference.semi_major * 1.45f,
                           reference.semi_minor * 1.45f),
                reference.angle_deg, 0.0, 360.0, cv::Scalar(0, 255, 255), 1);
  }
  if (detection.valid) {
    cv::ellipse(overlay,
                cv::Point(cvRound(detection.disc_ellipse.center.x),
                          cvRound(detection.disc_ellipse.center.y)),
                cv::Size2f(detection.disc_ellipse.semi_major,
                           detection.disc_ellipse.semi_minor),
                detection.disc_ellipse.angle_deg, 0.0, 360.0,
                cv::Scalar(0, 0, 255), 2);
    cv::circle(overlay,
               cv::Point(cvRound(detection.disc_ellipse.center.x),
                         cvRound(detection.disc_ellipse.center.y)),
               2, cv::Scalar(0, 0, 255), -1);
  }
  const cv::Scalar color = detection.valid ? cv::Scalar(0, 255, 0)
                                           : cv::Scalar(0, 0, 255);
  const char* text = detection.valid ? "FOUND" : "MISS";
  cv::putText(overlay, text, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1.2,
              color, 3);
  return overlay;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) return std::string();
      return argv[++i];
    };
    if (arg == "--video") {
      options.video = value();
    } else if (arg == "--config") {
      options.config_path = value();
    } else if (arg == "--frames") {
      options.frames_text = value();
    } else if (arg == "--out") {
      options.out_dir = value();
    } else if (arg == "--csv") {
      options.csv_path = value();
    } else if (arg == "--max-frames") {
      options.max_frames = std::atoi(value().c_str());
    } else if (arg == "--dump-every") {
      options.dump_every = std::atoi(value().c_str());
    } else if (arg == "--all") {
      options.all_frames = true;
    } else if (arg == "--check") {
      options.check = true;
    } else if (arg == "--roi") {
      options.roi = true;
    } else if (arg == "--compare-roi") {
      options.compare_roi = true;
    } else if (arg == "--no-debug") {
      options.no_debug = true;
    } else if (arg == "--verbose") {
      options.verbose = true;
    } else if (arg == "--help" || arg == "-h") {
      std::printf(
          "usage: glowtrack_devdet --video <file> [--config <yaml>]\n"
          "       [--frames 0,120,300] [--out dir] [--all] [--check] [--roi]\n"
          "       [--compare-roi] [--csv path] [--max-frames N] "
          "[--dump-every N] [--no-debug] [--verbose]\n");
      return 0;
    } else {
      std::fprintf(stderr, "error: unknown argument '%s' (try --help)\n",
                   arg.c_str());
      return 1;
    }
  }
  if (options.video.empty()) {
    std::fprintf(stderr, "error: --video is required\n");
    return 1;
  }

  const glowtrack::DetectionConfig detector_config =
      loadDetectorConfig(options.config_path);
  glowtrack::TargetGeometry target;  // defaults; only used for scaling output

  cv::VideoCapture capture(options.video);
  if (!capture.isOpened()) {
    std::fprintf(stderr, "error: cannot open video %s\n", options.video.c_str());
    return 1;
  }
  glowtrack::TargetDetector detector(detector_config, target);
  detector.setDebugEnabled(!options.no_debug);

  if (!options.all_frames) {
    // -----------------------------------------------------------------
    // Frame dump mode
    // -----------------------------------------------------------------
    std::filesystem::create_directories(options.out_dir);
    const std::vector<int> frames = parseFrameList(options.frames_text);
    int index = 0;
    cv::Mat frame;
    while (capture.read(frame)) {
      if (!containsFrame(frames, index)) {
        ++index;
        continue;
      }
      const glowtrack::Detection detection = detector.detect(frame, index);
      cv::Mat debug = renderOverlay(frame, detection, ReferenceDisc(), false,
                                    cv::Rect());
      char name[160];
      std::snprintf(name, sizeof(name), "frame_%05d.png", index);
      cv::imwrite((std::filesystem::path(options.out_dir) / name).string(),
                  debug);
      if (!detector.debugMask().empty()) {
        std::snprintf(name, sizeof(name), "mask_%05d.png", index);
        cv::imwrite((std::filesystem::path(options.out_dir) / name).string(),
                    detector.debugMask());
      }
      if (!detector.debugCandidates().empty()) {
        std::snprintf(name, sizeof(name), "cand_%05d.png", index);
        cv::imwrite((std::filesystem::path(options.out_dir) / name).string(),
                    detector.debugCandidates());
      }
      std::printf(
          "frame %d: %s center=(%.1f, %.1f) axes=(%.1f, %.1f) angle=%.1f "
          "score=%.3f glow_area=%.0f\n",
          index, detection.valid ? "FOUND" : "MISS",
          detection.disc_ellipse.center.x, detection.disc_ellipse.center.y,
          detection.disc_ellipse.semi_major, detection.disc_ellipse.semi_minor,
          detection.disc_ellipse.angle_deg, detection.score, detection.area_px);
      printCandidateInfo(detector);
      ++index;
    }
    std::printf("wrote debug images to %s\n", options.out_dir.c_str());
    return 0;
  }

  // -------------------------------------------------------------------
  // Whole-video mode: recall / timing / reference check.
  // -------------------------------------------------------------------
  std::FILE* csv = nullptr;
  if (!options.csv_path.empty()) {
    csv = std::fopen(options.csv_path.c_str(), "w");
    if (csv != nullptr) {
      std::fprintf(csv,
                   "frame,valid,cx,cy,a,b,angle,score,glow_area,ms,"
                   "ring,wf,core,fill,V,S,ref_ok,ref_cx,ref_cy,dist_px,"
                   "match\n");
    } else {
      std::fprintf(stderr, "warning: cannot write csv %s\n",
                   options.csv_path.c_str());
    }
  }

  std::vector<double> times_ms;
  times_ms.reserve(1024);
  std::vector<int> misses;
  std::vector<int> false_positives;
  int index = 0;
  int processed = 0;
  int detected = 0;
  int matched = 0;
  int reference_found = 0;
  // ROI-vs-full-search agreement counters (--compare-roi).
  int roi_compared = 0;
  int roi_valid_mismatch = 0;
  float roi_max_center_diff = 0.f;
  float roi_max_axis_rel_diff = 0.f;
  cv::Rect roi;
  cv::Mat frame;
  while (capture.read(frame)) {
    if (options.max_frames > 0 && processed >= options.max_frames) {
      break;
    }
    cv::Rect search_roi;
    if (options.roi && roi.width > 0 && roi.height > 0) {
      search_roi = roi;
    }
    const auto start = std::chrono::steady_clock::now();
    const glowtrack::Detection detection =
        detector.detect(frame, search_roi, index);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
    times_ms.push_back(elapsed_ms);
    if (detection.valid) {
      ++detected;
      const float radius = detection.disc_ellipse.mean_radius();
      const float half = detector_config.roi_padding_scale * radius +
                         static_cast<float>(detector_config.roi_min_size_px) *
                             0.5f;
      roi = cv::Rect(cvRound(detection.disc_ellipse.center.x - half),
                     cvRound(detection.disc_ellipse.center.y - half),
                     cvRound(2.f * half), cvRound(2.f * half)) &
            cv::Rect(0, 0, frame.cols, frame.rows);
    } else {
      roi = cv::Rect();
    }

    // ROI search must return the same best candidate as the full-frame search.
    if (options.compare_roi) {
      cv::Rect derived_roi;
      if (detection.valid) {
        const float half = detector_config.roi_padding_scale *
                               detection.disc_ellipse.mean_radius() +
                           static_cast<float>(detector_config.roi_min_size_px) *
                               0.5f;
        derived_roi = cv::Rect(
            cvRound(detection.disc_ellipse.center.x - half),
            cvRound(detection.disc_ellipse.center.y - half), cvRound(2.f * half),
            cvRound(2.f * half));
      }
      derived_roi &= cv::Rect(0, 0, frame.cols, frame.rows);
      if (derived_roi.width > 0 && derived_roi.height > 0) {
        const glowtrack::Detection roi_detection =
            detector.detect(frame, derived_roi, index);
        if (roi_detection.valid != detection.valid) {
          ++roi_valid_mismatch;
        } else if (roi_detection.valid) {
          ++roi_compared;
          const float dx = roi_detection.disc_ellipse.center.x -
                           detection.disc_ellipse.center.x;
          const float dy = roi_detection.disc_ellipse.center.y -
                           detection.disc_ellipse.center.y;
          const float center_diff = std::sqrt(dx * dx + dy * dy);
          const float major_ref =
              std::max(1.f, detection.disc_ellipse.semi_major);
          const float minor_ref =
              std::max(1.f, detection.disc_ellipse.semi_minor);
          const float major_rel =
              std::fabs(roi_detection.disc_ellipse.semi_major - major_ref) /
              major_ref;
          const float minor_rel =
              std::fabs(roi_detection.disc_ellipse.semi_minor - minor_ref) /
              minor_ref;
          roi_max_center_diff = std::max(roi_max_center_diff, center_diff);
          roi_max_axis_rel_diff =
              std::max(roi_max_axis_rel_diff, std::max(major_rel, minor_rel));
        }
      }
    }

    ReferenceDisc reference;
    bool reference_ok = false;
    if (options.check) {
      reference = findReferenceDisc(frame, detector_config);
      reference_ok = reference.valid;
      if (reference_ok) {
        ++reference_found;
      }
    }
    float distance = -1.f;
    bool is_match = false;
    if (reference_ok && detection.valid) {
      const float dx = detection.disc_ellipse.center.x - reference.center.x;
      const float dy = detection.disc_ellipse.center.y - reference.center.y;
      distance = std::sqrt(dx * dx + dy * dy);
      const float tolerance =
          std::max(12.f, 0.5f * std::max(reference.semi_major,
                                         detection.disc_ellipse.semi_major));
      is_match = distance <= tolerance;
      if (is_match) {
        ++matched;
      } else {
        false_positives.push_back(index);
      }
    }
    if (options.check) {
      if (reference_ok && (!detection.valid || !is_match)) {
        misses.push_back(index);
      }
      if (!reference_ok && detection.valid) {
        // No independent reference available: cannot classify; ignore.
      }
    } else if (!detection.valid) {
      misses.push_back(index);
    }

    if (options.dump_every > 0 && (processed % options.dump_every) == 0) {
      dumpDebugImages(detector, frame, index, options.out_dir);
      cv::imwrite(
          (std::filesystem::path(options.out_dir) /
           ("overlay_" + std::to_string(index) + ".png"))
              .string(),
          renderOverlay(frame, detection, reference, options.roi, search_roi));
    }
    if (csv != nullptr) {
      const glowtrack::TargetDetector::CandidateInfo* best_info = nullptr;
      for (const auto& info : detector.debugCandidateInfo()) {
        if (best_info == nullptr || info.score > best_info->score) {
          best_info = &info;
        }
      }
      std::fprintf(csv, "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.0f,%.3f,",
                   index, detection.valid ? 1 : 0,
                   detection.disc_ellipse.center.x,
                   detection.disc_ellipse.center.y,
                   detection.disc_ellipse.semi_major,
                   detection.disc_ellipse.semi_minor,
                   detection.disc_ellipse.angle_deg, detection.score,
                   detection.area_px, elapsed_ms);
      if (best_info != nullptr) {
        std::fprintf(csv, "%.3f,%.3f,%.3f,%.3f,%.0f,%.0f,",
                     best_info->ring_ratio, best_info->white_frac,
                     best_info->core_frac, best_info->fill_ratio,
                     best_info->mean_v, best_info->mean_s);
      } else {
        std::fprintf(csv, ",,,,,,");
      }
      std::fprintf(csv, "%d,", reference_ok ? 1 : 0);
      if (reference_ok) {
        std::fprintf(csv, "%.2f,%.2f,%.2f,%d", reference.center.x,
                     reference.center.y, distance, is_match ? 1 : 0);
      } else {
        std::fprintf(csv, ",,,");
      }
      std::fprintf(csv, "\n");
    }
    if (options.verbose) {
      std::printf("frame %5d %s c=(%.1f,%.1f) s=%.2f %.2f ms%s\n", index,
                  detection.valid ? "FOUND" : "MISS ",
                  detection.disc_ellipse.center.x,
                  detection.disc_ellipse.center.y, detection.score, elapsed_ms,
                  options.check && reference_ok
                      ? (is_match ? " match" : " FP")
                      : "");
      if (detection.valid) {
        printCandidateInfo(detector);
      }
    }
    ++index;
    ++processed;
  }
  if (csv != nullptr) {
    std::fclose(csv);
  }

  std::sort(times_ms.begin(), times_ms.end());
  auto percentile = [&times_ms](double p) {
    if (times_ms.empty()) return 0.0;
    const size_t idx = static_cast<size_t>(
        std::min<double>(static_cast<double>(times_ms.size() - 1),
                         p * static_cast<double>(times_ms.size() - 1) + 0.5));
    return times_ms[idx];
  };
  double sum = 0.0;
  for (double t : times_ms) sum += t;

  std::printf("===== %s =====\n", options.video.c_str());
  std::printf("frames processed : %d\n", processed);
  std::printf("detected         : %d (%.2f %%)\n", detected,
              100.0 * detected / std::max(processed, 1));
  std::printf("detector time    : mean %.2f ms | p50 %.2f | p95 %.2f | max %.2f "
              "ms\n",
              sum / std::max<double>(1.0, static_cast<double>(times_ms.size())),
              percentile(0.50), percentile(0.95), percentile(0.99));
  if (options.check) {
    std::printf("reference discs  : %d\n", reference_found);    std::printf("matched          : %d / %d reference (%.2f %%)\n", matched,
                std::max(reference_found, 1),
                100.0 * matched / std::max(reference_found, 1));
    std::printf("false positives  : %d (%.3f %% of frames)\n",
                static_cast<int>(false_positives.size()),
                100.0 * static_cast<double>(false_positives.size()) /
                    std::max(processed, 1));
    std::printf("misses           : %d (%.3f %% of frames)\n",
                static_cast<int>(misses.size()),
                100.0 * static_cast<double>(misses.size()) /
                    std::max(processed, 1));
    if (!misses.empty()) {
      std::printf("  miss frames  :");
      for (size_t i = 0; i < misses.size() && i < 40; ++i) {
        std::printf(" %d", misses[i]);
      }
      std::printf("%s\n", misses.size() > 40 ? " ..." : "");
    }
    if (!false_positives.empty()) {
      std::printf("  FP frames    :");
      for (size_t i = 0; i < false_positives.size() && i < 40; ++i) {
        std::printf(" %d", false_positives[i]);
      }
      std::printf("%s\n", false_positives.size() > 40 ? " ..." : "");
    }
  }
  if (!options.csv_path.empty()) {
    std::printf("csv              : %s\n", options.csv_path.c_str());
  }
  if (options.compare_roi) {
    std::printf(
        "ROI-vs-full      : %d frames compared | max center diff %.2f px | "
        "max axis rel diff %.2f %% | valid mismatches %d\n",
        roi_compared, roi_max_center_diff, 100.0 * roi_max_axis_rel_diff,
        roi_valid_mismatch);
  }
  return detected > 0 ? 0 : 1;
}
