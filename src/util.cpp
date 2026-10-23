#include "glowtrack/util.hpp"

#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <vector>

namespace glowtrack {

double normalizeAngle(double angle_rad) {
  // atan2 style wrapping: (-pi, pi]
  double a = std::fmod(angle_rad + kPi, 2.0 * kPi);
  if (a < 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

double normalizeAnglePositive(double angle_rad) {
  double a = std::fmod(angle_rad, 2.0 * kPi);
  if (a < 0.0) {
    a += 2.0 * kPi;
  }
  return a;
}

double angleDifference(double a, double b) { return normalizeAngle(a - b); }

std::string formatString(const char* fmt, ...) {
  va_list args;
  va_start(args, fmt);
  va_list args_copy;
  va_copy(args_copy, args);
  const int needed = std::vsnprintf(nullptr, 0, fmt, args_copy);
  va_end(args_copy);
  if (needed <= 0) {
    va_end(args);
    return std::string();
  }
  std::vector<char> buffer(static_cast<size_t>(needed) + 1);
  std::vsnprintf(buffer.data(), buffer.size(), fmt, args);
  va_end(args);
  return std::string(buffer.data(), static_cast<size_t>(needed));
}

double nowSeconds() {
  using clock = std::chrono::steady_clock;
  static const auto start = clock::now();
  return std::chrono::duration<double>(clock::now() - start).count();
}

bool ensureParentDirectory(const std::string& path) {
  try {
    const std::filesystem::path p(path);
    const auto parent = p.parent_path();
    if (parent.empty()) {
      return true;
    }
    if (std::filesystem::exists(parent)) {
      return true;
    }
    return std::filesystem::create_directories(parent);
  } catch (const std::exception&) {
    return false;
  }
}

bool isIntegerIndex(const std::string& text) {
  if (text.empty()) {
    return false;
  }
  return std::all_of(text.begin(), text.end(),
                     [](unsigned char c) { return std::isdigit(c) != 0; });
}

}  // namespace glowtrack
