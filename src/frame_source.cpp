// Frame sources: video file, image sequence, live USB camera, OpenMV serial
// JPEG stream.
//
// OpenMvSerial framing (defined with the Lead, documented in docs/protocol.md
// section 2, produced by openmv/openmv_cam_stream.py):
//
//   offset 0  u8    0xA5 (sof1)
//   offset 1  u8    0x5A (sof2)
//   offset 2  u8    version (= 1)
//   offset 3  u8    flags   (reserved, 0)
//   offset 4  u8    seq     (frame counter, wraps at 256)
//   offset 5  u16   width   (little endian)
//   offset 7  u16   height  (little endian)
//   offset 9  u32   jpeg_len(little endian)
//   offset 13 u8[]  payload (JPEG bytes, jpeg_len of them)
//   offset 13+len u16 crc16 (little endian, CRC-16/CCITT-FALSE over
//                            version..payload, i.e. bytes [2, 13+jpeg_len))
//
// The reader below synchronises on A5 5A, validates the header, length and CRC,
// then decodes the JPEG with cv::imdecode. Bad frames are dropped by sliding the
// window one byte and resynchronising.

#include "glowtrack/frame_source.hpp"

#include "glowtrack/kalman_filter.hpp"  // crc16Ccitt
#include "glowtrack/output_sink.hpp"    // SerialPort
#include "glowtrack/util.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace glowtrack {
namespace {

using SteadyClock = std::chrono::steady_clock;

double elapsedSeconds(SteadyClock::time_point start) {
  return std::chrono::duration<double>(SteadyClock::now() - start).count();
}

std::string lowerExtension(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char c) {
                   return static_cast<char>(std::tolower(c));
                 });
  return extension;
}

bool isImageFile(const std::filesystem::path& path) {
  const std::string extension = lowerExtension(path);
  return extension == ".png" || extension == ".jpg" || extension == ".jpeg" ||
         extension == ".bmp";
}

// Natural-order comparison of two strings: numeric runs compare numerically so
// that frame_2 sorts before frame_10. Plain lexicographic ordering is kept for
// the non-numeric remainder, which gives a strict weak ordering for unique
// names.
bool naturalLessText(const std::string& a, const std::string& b) {
  size_t i = 0;
  size_t j = 0;
  while (i < a.size() && j < b.size()) {
    const bool digit_a = std::isdigit(static_cast<unsigned char>(a[i])) != 0;
    const bool digit_b = std::isdigit(static_cast<unsigned char>(b[j])) != 0;
    if (digit_a && digit_b) {
      size_t end_a = i;
      size_t end_b = j;
      while (end_a < a.size() &&
             std::isdigit(static_cast<unsigned char>(a[end_a])) != 0) {
        ++end_a;
      }
      while (end_b < b.size() &&
             std::isdigit(static_cast<unsigned char>(b[end_b])) != 0) {
        ++end_b;
      }
      // Compare the digit runs numerically (skip leading zeros).
      size_t num_a = i;
      size_t num_b = j;
      while (num_a + 1 < end_a && a[num_a] == '0') {
        ++num_a;
      }
      while (num_b + 1 < end_b && b[num_b] == '0') {
        ++num_b;
      }
      const size_t len_a = end_a - num_a;
      const size_t len_b = end_b - num_b;
      if (len_a != len_b) {
        return len_a < len_b;
      }
      const int numeric = a.compare(num_a, len_a, b, num_b, len_b);
      if (numeric != 0) {
        return numeric < 0;
      }
      i = end_a;
      j = end_b;
      continue;
    }
    if (a[i] != b[j]) {
      return a[i] < b[j];
    }
    ++i;
    ++j;
  }
  return (a.size() - i) < (b.size() - j);
}

// ---------------------------------------------------------------------------
// Video file
// ---------------------------------------------------------------------------
class VideoFileSource final : public FrameSource {
 public:
  bool open(const std::string& path, double fallback_fps,
            std::string* error) {
    namespace fs = std::filesystem;
    std::error_code code;
    if (!fs::exists(path, code)) {
      if (error != nullptr) {
        *error = "video file not found: " + path;
      }
      return false;
    }
    configured_fps_ = fallback_fps > 1e-6 ? fallback_fps : 30.0;
    if (!capture_.open(path)) {
      if (error != nullptr) {
        *error = "cannot open video file: " + path;
      }
      return false;
    }
    const double capture_fps =
        capture_.get(cv::CAP_PROP_FPS);  // NOLINT(google-readability-casting)
    fps_ = (std::isfinite(capture_fps) && capture_fps > 0.5 &&
            capture_fps < 1000.0)
               ? capture_fps
               : configured_fps_;
    const double frames = capture_.get(cv::CAP_PROP_FRAME_COUNT);
    frame_count_ = (std::isfinite(frames) && frames > 0.0)
                       ? static_cast<int>(frames + 0.5)
                       : -1;
    return true;
  }

  bool read(cv::Mat& bgr, double& timestamp_s) override {
    if (!capture_.read(bgr) || bgr.empty()) {
      return false;
    }
    double timestamp = std::numeric_limits<double>::quiet_NaN();
    const double position_ms = capture_.get(cv::CAP_PROP_POS_MSEC);
    if (std::isfinite(position_ms) && position_ms >= 0.0) {
      timestamp = position_ms / 1000.0;
    }
    const double step = (fps_ > 1e-6) ? (1.0 / fps_) : (1.0 / 30.0);
    if (!std::isfinite(timestamp)) {
      timestamp = static_cast<double>(index_) * step;
    }
    // Some backends report 0 or repeat the same position: keep timestamps
    // strictly increasing so that the tracker never sees dt <= 0.
    if (has_last_timestamp_ && timestamp <= last_timestamp_s_) {
      timestamp = last_timestamp_s_ + 0.5 * step;
    }
    last_timestamp_s_ = timestamp;
    has_last_timestamp_ = true;
    ++index_;
    timestamp_s = timestamp;
    return true;
  }

  int frameCount() const override { return frame_count_; }
  double fps() const override { return fps_; }

 private:
  cv::VideoCapture capture_;
  double configured_fps_ = 30.0;
  double fps_ = 0.0;
  int frame_count_ = -1;
  int index_ = 0;
  bool has_last_timestamp_ = false;
  double last_timestamp_s_ = 0.0;
};

// ---------------------------------------------------------------------------
// Image sequence
// ---------------------------------------------------------------------------
class ImageSequenceSource final : public FrameSource {
 public:
  bool open(const std::string& directory, double fps, std::string* error) {
    namespace fs = std::filesystem;
    std::error_code code;
    if (!fs::is_directory(directory, code)) {
      if (error != nullptr) {
        *error = "image sequence directory not found: " + directory;
      }
      return false;
    }
    const fs::path root(directory);

    // Prefer the flat layout (png/jpg files directly inside `directory`).
    collectImages(root, /*recursive=*/false);
    if (files_.empty()) {
      // Fall back to the nested layout used by tools/synth_dataset.cpp:
      //   <scenario>/frames/frame_00000.png (+ gt.csv / meta.yaml at the top).
      collectImages(root, /*recursive=*/true);
    }
    if (files_.empty()) {
      if (error != nullptr) {
        *error = "no png/jpg/bmp images found (searched recursively) in: " +
                 directory;
      }
      return false;
    }
    // Natural order; for nested layouts compare the path relative to the
    // sequence root so sub-directories sort deterministically as well.
    std::sort(files_.begin(), files_.end(),
              [&root](const fs::path& a, const fs::path& b) {
                return naturalLessText(a.lexically_relative(root).generic_string(),
                                       b.lexically_relative(root).generic_string());
              });
    fps_ = (fps > 1e-6) ? fps : 30.0;
    return true;
  }

  bool read(cv::Mat& bgr, double& timestamp_s) override {
    if (index_ >= files_.size()) {
      return false;
    }
    cv::Mat image = cv::imread(files_[index_].string(), cv::IMREAD_COLOR);
    if (image.empty()) {
      // Skip unreadable frame and report end-of-stream; the CLI treats a
      // false return as "stop".
      return false;
    }
    bgr = image;
    timestamp_s = static_cast<double>(index_) / fps_;
    ++index_;
    return true;
  }

  int frameCount() const override {
    return static_cast<int>(files_.size());
  }
  double fps() const override { return fps_; }

 private:
  void collectImages(const std::filesystem::path& root, bool recursive) {
    namespace fs = std::filesystem;
    std::error_code code;
    if (recursive) {
      fs::recursive_directory_iterator iterator(
          root, fs::directory_options::skip_permission_denied, code);
      const fs::recursive_directory_iterator end;
      while (!code && iterator != end) {
        std::error_code entry_code;
        if (iterator->is_regular_file(entry_code) && !entry_code &&
            isImageFile(iterator->path())) {
          files_.push_back(iterator->path());
        }
        iterator.increment(code);
      }
      return;
    }
    for (const auto& entry : fs::directory_iterator(root, code)) {
      if (code) {
        break;
      }
      std::error_code entry_code;
      if (!entry.is_regular_file(entry_code) || entry_code) {
        continue;
      }
      if (isImageFile(entry.path())) {
        files_.push_back(entry.path());
      }
    }
  }

  std::vector<std::filesystem::path> files_;
  size_t index_ = 0;
  double fps_ = 30.0;
};

// ---------------------------------------------------------------------------
// Live camera
// ---------------------------------------------------------------------------
class CameraSource final : public FrameSource {
 public:
  bool open(const std::string& path, double fallback_fps, std::string* error) {
    int index = 0;
    try {
      size_t consumed = 0;
      index = std::stoi(path, &consumed);
      if (consumed != path.size()) {
        if (error != nullptr) {
          *error = "camera source path must be an integer index, got '" + path +
                   "'";
        }
        return false;
      }
    } catch (const std::exception&) {
      if (error != nullptr) {
        *error = "camera source path must be an integer index, got '" + path +
                 "'";
      }
      return false;
    }
    configured_fps_ = fallback_fps > 1e-6 ? fallback_fps : 30.0;
    bool opened = false;
#if defined(_WIN32)
    opened = capture_.open(index, cv::CAP_DSHOW);
    if (!opened) {
      opened = capture_.open(index);
    }
#else
    opened = capture_.open(index);
#endif
    if (!opened || !capture_.isOpened()) {
      if (error != nullptr) {
        *error = "cannot open camera index " + std::to_string(index);
      }
      return false;
    }
    const double capture_fps = capture_.get(cv::CAP_PROP_FPS);
    fps_ = (std::isfinite(capture_fps) && capture_fps > 0.5 &&
            capture_fps < 1000.0)
               ? capture_fps
               : configured_fps_;
    start_ = SteadyClock::now();
    return true;
  }

  bool read(cv::Mat& bgr, double& timestamp_s) override {
    if (!capture_.read(bgr) || bgr.empty()) {
      return false;
    }
    timestamp_s = elapsedSeconds(start_);
    return true;
  }

  double fps() const override { return fps_; }
  bool isLive() const override { return true; }

 private:
  cv::VideoCapture capture_;
  double configured_fps_ = 30.0;
  double fps_ = 30.0;
  SteadyClock::time_point start_{};
};

// ---------------------------------------------------------------------------
// OpenMV JPEG stream over a serial port (framing documented at the top).
// ---------------------------------------------------------------------------
class OpenMvSerialSource final : public FrameSource {
 public:
  bool open(const FrameSourceConfig& config, std::string* error) {
    if (config.path.empty()) {
      if (error != nullptr) {
        *error = "OpenMV serial source requires --openmv <port>";
      }
      return false;
    }
    std::string serial_error;
    port_ = SerialPort::open(config.path, config.serial_baud, &serial_error);
    if (!port_) {
      if (error != nullptr) {
        *error = "cannot open OpenMV serial port '" + config.path +
                 "': " + serial_error;
      }
      return false;
    }
    configured_fps_ = config.fps > 1e-6 ? config.fps : 30.0;
    buffer_.reserve(1u << 16);
    start_ = SteadyClock::now();
    return true;
  }

  bool read(cv::Mat& bgr, double& timestamp_s) override {
    if (!port_ || !port_->isOpen()) {
      return false;
    }
    const auto deadline =
        SteadyClock::now() + std::chrono::milliseconds(kFrameTimeoutMs);
    while (SteadyClock::now() < deadline) {
      if (tryDecode(&bgr)) {
        timestamp_s = elapsedSeconds(start_);
        return true;
      }
      uint8_t chunk[4096];
      const int received = port_->readSome(chunk, sizeof(chunk));
      if (received > 0) {
        buffer_.insert(buffer_.end(), chunk, chunk + received);
        if (buffer_.size() > kMaxBufferBytes) {
          buffer_.erase(buffer_.begin(),
                        buffer_.begin() +
                            static_cast<std::ptrdiff_t>(buffer_.size() -
                                                        kMaxBufferBytes));
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return false;
  }

  double fps() const override { return configured_fps_; }
  bool isLive() const override { return true; }

 private:
  static constexpr int kFrameTimeoutMs = 2000;
  static constexpr size_t kMaxBufferBytes = 8u << 20;  // 8 MiB
  static constexpr uint32_t kMaxJpegBytes = 4u << 20;
  static constexpr size_t kHeaderBytes = 13;

  static uint16_t loadLe16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
  }
  static uint32_t loadLe32(const uint8_t* data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
  }

  // Attempts to pull one complete, CRC-valid frame out of the buffer.
  bool tryDecode(cv::Mat* bgr) {
    for (;;) {
      // Resynchronise on the A5 5A header.
      size_t offset = 0;
      while (offset + 1 < buffer_.size() &&
             !(buffer_[offset] == 0xA5 && buffer_[offset + 1] == 0x5A)) {
        ++offset;
      }
      if (offset > 0) {
        buffer_.erase(buffer_.begin(),
                      buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
      }
      if (buffer_.size() < kHeaderBytes) {
        return false;
      }
      const uint8_t version = buffer_[2];
      const uint16_t width = loadLe16(&buffer_[5]);
      const uint16_t height = loadLe16(&buffer_[7]);
      const uint32_t jpeg_length = loadLe32(&buffer_[9]);
      const bool plausible =
          version >= 1 && version <= 16 && width > 0 && width <= 8192 &&
          height > 0 && height <= 8192 && jpeg_length > 128 &&
          jpeg_length <= kMaxJpegBytes;
      if (!plausible) {
        buffer_.erase(buffer_.begin());  // drop one byte and resync
        continue;
      }
      const size_t total = kHeaderBytes + static_cast<size_t>(jpeg_length) + 2;
      if (buffer_.size() < total) {
        return false;  // wait for more bytes
      }
      const uint16_t received_crc =
          loadLe16(&buffer_[kHeaderBytes + jpeg_length]);
      const uint16_t expected_crc = crc16Ccitt(
          &buffer_[2], 11 + static_cast<size_t>(jpeg_length));
      if (received_crc != expected_crc) {
        buffer_.erase(buffer_.begin());  // drop one byte and resync
        continue;
      }
      std::vector<uint8_t> jpeg(buffer_.begin() + kHeaderBytes,
                                buffer_.begin() + kHeaderBytes + jpeg_length);
      buffer_.erase(buffer_.begin(),
                    buffer_.begin() + static_cast<std::ptrdiff_t>(total));
      cv::Mat decoded = cv::imdecode(jpeg, cv::IMREAD_COLOR);
      if (!decoded.empty()) {
        *bgr = decoded;
        return true;
      }
      // Corrupt JPEG payload: keep scanning for the next header.
    }
  }

  std::unique_ptr<SerialPort> port_;
  std::vector<uint8_t> buffer_;
  double configured_fps_ = 30.0;
  SteadyClock::time_point start_{};
};

}  // namespace

std::unique_ptr<FrameSource> FrameSource::create(
    const FrameSourceConfig& config, std::string* error) {
  std::string local_error;
  std::string* message = (error != nullptr) ? error : &local_error;
  message->clear();

  switch (config.type) {
    case FrameSourceConfig::Type::VideoFile: {
      auto source = std::make_unique<VideoFileSource>();
      if (!source->open(config.path, config.fps, message)) {
        return nullptr;
      }
      return source;
    }
    case FrameSourceConfig::Type::ImageSequence: {
      auto source = std::make_unique<ImageSequenceSource>();
      if (!source->open(config.path, config.fps, message)) {
        return nullptr;
      }
      return source;
    }
    case FrameSourceConfig::Type::Camera: {
      auto source = std::make_unique<CameraSource>();
      if (!source->open(config.path, config.fps, message)) {
        return nullptr;
      }
      return source;
    }
    case FrameSourceConfig::Type::OpenMvSerial: {
      auto source = std::make_unique<OpenMvSerialSource>();
      if (!source->open(config, message)) {
        return nullptr;
      }
      return source;
    }
  }
  *message = "unknown frame source type";
  return nullptr;
}

}  // namespace glowtrack
