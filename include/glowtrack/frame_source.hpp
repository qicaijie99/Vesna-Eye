#pragma once
// Frame sources: video file, image sequence, USB camera, OpenMV serial stream.

#include <opencv2/core.hpp>

#include <memory>
#include <string>

namespace glowtrack {

struct FrameSourceConfig {
  enum class Type {
    VideoFile,     // path = video file (mp4/avi/...)
    ImageSequence, // path = directory with frames (png/jpg), sorted by name
    Camera,        // path = camera index as string
    OpenMvSerial,  // path = COM port of an OpenMV camera streaming JPEG frames
  };

  Type type = Type::VideoFile;
  std::string path;
  double fps = 30.0;       // image sequence timestamp step
  int serial_baud = 921600;  // OpenMV serial stream baud rate
};

class FrameSource {
 public:
  virtual ~FrameSource() = default;

  // Creates a source. On failure returns nullptr and fills `error`.
  static std::unique_ptr<FrameSource> create(const FrameSourceConfig& config,
                                             std::string* error);

  // Reads the next frame. Returns false at end of stream / on error.
  // `timestamp_s` is the capture timestamp in seconds (may be relative).
  virtual bool read(cv::Mat& bgr, double& timestamp_s) = 0;

  virtual int frameCount() const { return -1; }
  virtual double fps() const { return 0.0; }
  virtual bool isLive() const { return false; }
};

}  // namespace glowtrack
