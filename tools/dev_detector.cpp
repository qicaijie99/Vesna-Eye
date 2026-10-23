// Developer helper for the detector workstream: dump detection debug images for
// selected frames of a video so the thresholds can be tuned quickly.
//
//   glowtrack_devdet --video <file> --config <yaml> --frames 0,120,300 --out dir
//
// Owned by the detector workstream -- extend freely.

#include "glowtrack/config.hpp"
#include "glowtrack/detector.hpp"
#include "glowtrack/util.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace {

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

}  // namespace

int main(int argc, char** argv) {
  std::string video;
  std::string config_path = "config/video_720p_phone.yaml";
  std::string out_dir = "results/devdet";
  std::string frames_text = "0";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) return std::string();
      return argv[++i];
    };
    if (arg == "--video") {
      video = value();
    } else if (arg == "--config") {
      config_path = value();
    } else if (arg == "--frames") {
      frames_text = value();
    } else if (arg == "--out") {
      out_dir = value();
    } else if (arg == "--help" || arg == "-h") {
      std::printf(
          "usage: glowtrack_devdet --video <file> [--config <yaml>] "
          "[--frames 0,120,300] [--out dir]\n");
      return 0;
    }
  }
  if (video.empty()) {
    std::fprintf(stderr, "error: --video is required\n");
    return 1;
  }

  glowtrack::RuntimeConfig config;
  try {
    config = glowtrack::loadRuntimeConfig(config_path);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "warning: config load failed (%s), using defaults\n",
                 e.what());
  }

  cv::VideoCapture capture(video);
  if (!capture.isOpened()) {
    std::fprintf(stderr, "error: cannot open video %s\n", video.c_str());
    return 1;
  }
  std::filesystem::create_directories(out_dir);
  glowtrack::TargetDetector detector(config.detector, config.target);
  const std::vector<int> frames = parseFrameList(frames_text);

  int index = 0;
  cv::Mat frame;
  while (capture.read(frame)) {
    if (!containsFrame(frames, index)) {
      ++index;
      continue;
    }
    const glowtrack::Detection detection =
        detector.detect(frame, index);
    cv::Mat debug = frame.clone();
    if (detection.valid) {
      cv::ellipse(debug, detection.disc_ellipse.center,
                  cv::Size2f(detection.disc_ellipse.semi_major,
                             detection.disc_ellipse.semi_minor),
                  detection.disc_ellipse.angle_deg, 0, 360, cv::Scalar(0, 0, 255),
                  2);
    }
    cv::putText(debug,
                detection.valid ? "FOUND" : "MISS", cv::Point(10, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.2,
                detection.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                3);
    char name[128];
    std::snprintf(name, sizeof(name), "frame_%05d.png", index);
    cv::imwrite((std::filesystem::path(out_dir) / name).string(), debug);
    if (!detector.debugMask().empty()) {
      std::snprintf(name, sizeof(name), "mask_%05d.png", index);
      cv::imwrite((std::filesystem::path(out_dir) / name).string(),
                  detector.debugMask());
    }
    if (!detector.debugCandidates().empty()) {
      std::snprintf(name, sizeof(name), "cand_%05d.png", index);
      cv::imwrite((std::filesystem::path(out_dir) / name).string(),
                  detector.debugCandidates());
    }
    std::printf(
        "frame %d: %s center=(%.1f, %.1f) axes=(%.1f, %.1f) angle=%.1f "
        "score=%.2f\n",
        index, detection.valid ? "FOUND" : "MISS", detection.disc_ellipse.center.x,
        detection.disc_ellipse.center.y, detection.disc_ellipse.semi_major,
        detection.disc_ellipse.semi_minor, detection.disc_ellipse.angle_deg,
        detection.score);
    ++index;
  }
  return 0;
}
