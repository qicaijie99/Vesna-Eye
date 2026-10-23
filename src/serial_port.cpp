// Serial link to the lower computer: Win32 (CreateFile/ReadFile/WriteFile with
// COMMTIMEOUTS) on Windows, POSIX termios elsewhere.
//
// Packet format: fixed 19-byte TrackPacket, little endian, CRC-16/CCITT-FALSE
// over bytes [0, 17). See include/glowtrack/types.hpp and docs/protocol.md:
//
//   sof1=0xAA sof2=0x55 version=1 seq flags yaw_q pitch_q yaw_rate_q
//   pitch_rate_q distance_mm quality reserved crc16
//     flags bit0 valid, bit1 measured, bit2 predicted, bit3 acquiring,
//           bit4 tracking
//     yaw/pitch [1e-4 rad], rates [1e-3 rad/s], distance [mm] saturating u16,
//     quality 0..100.

#include "glowtrack/output_sink.hpp"

#include "glowtrack/kalman_filter.hpp"  // crc16Ccitt
#include "glowtrack/util.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace glowtrack {
namespace {

constexpr int kDefaultBaud = 115200;

int16_t quantizeInt16(double value, double quantum) {
  if (!std::isfinite(value) || quantum <= 0.0) {
    return 0;
  }
  const long long scaled =
      std::llround(value / quantum);  // not clamped yet -- intentional
  const long long clamped =
      std::min<long long>(32767, std::max<long long>(-32768, scaled));
  return static_cast<int16_t>(clamped);
}

uint8_t quantizeQuality(double quality) {
  if (!std::isfinite(quality)) {
    return 0;
  }
  const long long scaled = std::llround(quality * 100.0);
  return static_cast<uint8_t>(std::min<long long>(100, std::max<long long>(0, scaled)));
}

uint16_t quantizeDistanceMm(double distance_m) {
  if (!std::isfinite(distance_m) || distance_m <= 0.0) {
    return 0;
  }
  const long long millimetres = std::llround(distance_m * 1000.0);
  return static_cast<uint16_t>(
      std::min<long long>(65535, std::max<long long>(0, millimetres)));
}

// ---------------------------------------------------------------------------
// Win32 implementation
// ---------------------------------------------------------------------------
#if defined(_WIN32)

std::string lastWindowsError(const std::string& what) {
  const DWORD code = ::GetLastError();
  std::string description;
  LPSTR buffer = nullptr;
  const DWORD length = ::FormatMessageA(
      FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
          FORMAT_MESSAGE_IGNORE_INSERTS,
      nullptr, code, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
      reinterpret_cast<LPSTR>(&buffer), 0, nullptr);
  if (buffer != nullptr) {
    description.assign(buffer, length);
    ::LocalFree(buffer);
  }
  while (!description.empty() &&
         (description.back() == '\r' || description.back() == '\n' ||
          description.back() == ' ')) {
    description.pop_back();
  }
  return formatString("%s failed (Win32 error %lu: %s)", what.c_str(),
                      static_cast<unsigned long>(code),
                      description.c_str());
}

class Win32SerialPort final : public SerialPort {
 public:
  Win32SerialPort() = default;
  ~Win32SerialPort() override { close(); }

  bool openPort(const std::string& port, int baud, std::string* error) {
    std::string device = port;
    if (device.rfind("\\\\.\\", 0) != 0) {
      device = "\\\\.\\" + device;
    }
    handle_ = ::CreateFileA(device.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                            nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL,
                            nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
      if (error != nullptr) {
        *error = lastWindowsError("cannot open " + port);
      }
      return false;
    }

    DCB dcb;
    std::memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!::GetCommState(handle_, &dcb)) {
      if (error != nullptr) {
        *error = lastWindowsError("GetCommState(" + port + ")");
      }
      close();
      return false;
    }
    dcb.BaudRate = static_cast<DWORD>(baud > 0 ? baud : kDefaultBaud);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    // USB CDC serial devices (OpenMV, ST-Link, ...) usually need DTR asserted.
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if (!::SetCommState(handle_, &dcb)) {
      if (error != nullptr) {
        *error = lastWindowsError("SetCommState(" + port + ")");
      }
      close();
      return false;
    }

    // Fully asynchronous reads: ReadFile returns immediately with whatever is
    // available; readExact() implements the timeout by polling.
    COMMTIMEOUTS timeouts;
    std::memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 50;
    if (!::SetCommTimeouts(handle_, &timeouts)) {
      if (error != nullptr) {
        *error = lastWindowsError("SetCommTimeouts(" + port + ")");
      }
      close();
      return false;
    }
    ::SetupComm(handle_, 1 << 16, 1 << 16);
    ::PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    return true;
  }

  bool write(const uint8_t* data, size_t length) override {
    if (handle_ == INVALID_HANDLE_VALUE || data == nullptr || length == 0) {
      return false;
    }
    size_t written = 0;
    while (written < length) {
      const DWORD chunk_size = static_cast<DWORD>(
          std::min<size_t>(length - written, 4096));
      DWORD chunk_written = 0;
      if (!::WriteFile(handle_, data + written, chunk_size, &chunk_written,
                       nullptr) ||
          chunk_written == 0) {
        return false;
      }
      written += chunk_written;
    }
    return true;
  }

  int readSome(uint8_t* data, size_t length) override {
    if (handle_ == INVALID_HANDLE_VALUE || data == nullptr || length == 0) {
      return 0;
    }
    DWORD received = 0;
    const DWORD request =
        static_cast<DWORD>(std::min<size_t>(length, 4096));
    if (!::ReadFile(handle_, data, request, &received, nullptr)) {
      return -1;
    }
    return static_cast<int>(received);
  }

  bool readExact(uint8_t* data, size_t length, int timeout_ms) override {
    if (data == nullptr || length == 0) {
      return true;
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(0, timeout_ms));
    size_t received = 0;
    while (received < length && std::chrono::steady_clock::now() < deadline) {
      const int chunk = readSome(data + received, length - received);
      if (chunk > 0) {
        received += static_cast<size_t>(chunk);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return received == length;
  }

  void close() override {
    if (handle_ != INVALID_HANDLE_VALUE) {
      ::CloseHandle(handle_);
      handle_ = INVALID_HANDLE_VALUE;
    }
  }

  bool isOpen() const override { return handle_ != INVALID_HANDLE_VALUE; }

 private:
  HANDLE handle_ = INVALID_HANDLE_VALUE;
};

#else  // POSIX implementation

speed_t baudToSpeed(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return B115200;
  }
}

class PosixSerialPort final : public SerialPort {
 public:
  PosixSerialPort() = default;
  ~PosixSerialPort() override { close(); }

  bool openPort(const std::string& port, int baud, std::string* error) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      if (error != nullptr) {
        *error = formatString("cannot open %s: %s", port.c_str(),
                              std::strerror(errno));
      }
      return false;
    }
    termios options;
    std::memset(&options, 0, sizeof(options));
    if (::tcgetattr(fd_, &options) != 0) {
      if (error != nullptr) {
        *error = formatString("tcgetattr(%s): %s", port.c_str(),
                              std::strerror(errno));
      }
      close();
      return false;
    }
    ::cfmakeraw(&options);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CRTSCTS;
    const speed_t speed = baudToSpeed(baud > 0 ? baud : kDefaultBaud);
    ::cfsetispeed(&options, speed);
    ::cfsetospeed(&options, speed);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
      if (error != nullptr) {
        *error = formatString("tcsetattr(%s): %s", port.c_str(),
                              std::strerror(errno));
      }
      close();
      return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
  }

  bool write(const uint8_t* data, size_t length) override {
    if (fd_ < 0 || data == nullptr || length == 0) {
      return false;
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(500);
    size_t written = 0;
    while (written < length && std::chrono::steady_clock::now() < deadline) {
      const ssize_t chunk = ::write(fd_, data + written, length - written);
      if (chunk > 0) {
        written += static_cast<size_t>(chunk);
      } else if (chunk < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
                               errno == EINTR)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } else {
        return false;
      }
    }
    return written == length;
  }

  int readSome(uint8_t* data, size_t length) override {
    if (fd_ < 0 || data == nullptr || length == 0) {
      return 0;
    }
    const ssize_t received = ::read(fd_, data, length);
    if (received < 0) {
      return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }
    return static_cast<int>(received);
  }

  bool readExact(uint8_t* data, size_t length, int timeout_ms) override {
    if (data == nullptr || length == 0) {
      return true;
    }
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(std::max(0, timeout_ms));
    size_t received = 0;
    while (received < length && std::chrono::steady_clock::now() < deadline) {
      const int chunk = readSome(data + received, length - received);
      if (chunk > 0) {
        received += static_cast<size_t>(chunk);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
    return received == length;
  }

  void close() override {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool isOpen() const override { return fd_ >= 0; }

 private:
  int fd_ = -1;
};

#endif  // _WIN32

}  // namespace

std::unique_ptr<SerialPort> SerialPort::open(const std::string& port, int baud,
                                             std::string* error) {
  std::string local_error;
  std::string* message = (error != nullptr) ? error : &local_error;
  message->clear();
  if (port.empty()) {
    *message = "serial port name is empty";
    return nullptr;
  }
#if defined(_WIN32)
  auto serial = std::make_unique<Win32SerialPort>();
#else
  auto serial = std::make_unique<PosixSerialPort>();
#endif
  if (!serial->openPort(port, baud, message)) {
    return nullptr;
  }
  return serial;
}

bool sendTrackPacket(SerialPort& port, const TrackOutput& output, uint8_t seq,
                     std::string* error) {
  TrackPacket packet;
  packet.sof1 = 0xAA;
  packet.sof2 = 0x55;
  packet.version = 1;
  packet.seq = seq;
  packet.flags = 0;
  if (output.valid) {
    packet.flags |= 0x01;
  }
  if (output.measured) {
    packet.flags |= 0x02;
  }
  if (output.predicted) {
    packet.flags |= 0x04;
  }
  if (output.state == TrackState::Acquiring) {
    packet.flags |= 0x08;
  }
  if (output.state == TrackState::Tracking) {
    packet.flags |= 0x10;
  }
  packet.yaw_q = quantizeInt16(output.yaw, 1e-4);
  packet.pitch_q = quantizeInt16(output.pitch, 1e-4);
  packet.yaw_rate_q = quantizeInt16(output.yaw_rate, 1e-3);
  packet.pitch_rate_q = quantizeInt16(output.pitch_rate, 1e-3);
  packet.distance_mm = quantizeDistanceMm(output.distance_m);
  packet.quality = quantizeQuality(output.quality);
  packet.reserved = 0;
  packet.crc16 = crc16Ccitt(reinterpret_cast<const uint8_t*>(&packet),
                            sizeof(TrackPacket) - 2);

  if (!port.write(reinterpret_cast<const uint8_t*>(&packet),
                  sizeof(TrackPacket))) {
    if (error != nullptr) {
      *error = "serial write of track packet failed";
    }
    return false;
  }
  return true;
}

}  // namespace glowtrack
