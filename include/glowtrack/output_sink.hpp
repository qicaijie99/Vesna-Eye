#pragma once
// Outputs: CSV log, annotated overlay video, serial packet to lower computer.

#include "glowtrack/config.hpp"
#include "glowtrack/types.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <memory>
#include <string>

namespace glowtrack {

class CsvLogger {
 public:
  explicit CsvLogger(const std::string& path);
  ~CsvLogger();

  bool ok() const;
  const std::string& path() const;
  const std::string& lastError() const;

  void writeHeader();
  void log(const TrackOutput& output, const Detection& detection,
           const PoseEstimate& pose);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Draws the debug overlay on a copy of the input frame.
class OverlayRenderer {
 public:
  explicit OverlayRenderer(const RuntimeConfig& config);
  ~OverlayRenderer();

  cv::Mat render(const cv::Mat& bgr, const TrackOutput& output,
                 const Detection& detection, const PoseEstimate& pose,
                 const cv::Rect& search_roi, double processing_ms,
                 double fps);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class OverlayVideoWriter {
 public:
  OverlayVideoWriter(const std::string& path, double fps, int width, int height);
  ~OverlayVideoWriter();

  bool ok() const;
  const std::string& lastError() const;
  void write(const cv::Mat& frame);

 private:
  cv::VideoWriter writer_;
  std::string path_;
  std::string error_;
  bool ok_ = false;
};

// Serial link to the lower computer (Win32 / POSIX in serial_port.cpp).
class SerialPort {
 public:
  virtual ~SerialPort() = default;

  // Opens a serial port, e.g. "COM3" or "/dev/ttyUSB0". Returns nullptr and
  // fills `error` on failure.
  static std::unique_ptr<SerialPort> open(const std::string& port, int baud,
                                          std::string* error);

  virtual bool write(const uint8_t* data, size_t length) = 0;
  // Reads exactly `length` bytes with an overall timeout. Returns false on
  // timeout / error.
  virtual bool readExact(uint8_t* data, size_t length, int timeout_ms) = 0;
  // Non-blocking read of up to `length` bytes.
  virtual int readSome(uint8_t* data, size_t length) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
};

// Fills the CRC of a TrackPacket and sends it. Returns false on I/O error.
bool sendTrackPacket(SerialPort& port, const TrackOutput& output, uint8_t seq,
                     std::string* error);

}  // namespace glowtrack
