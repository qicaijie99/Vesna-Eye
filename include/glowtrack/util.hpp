#pragma once
// Small utilities shared by all modules: angle helpers, timing, string helpers.

#include "glowtrack/types.hpp"

#include <chrono>
#include <string>

namespace glowtrack {

constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kDeg2Rad = kPi / 180.0;

// Normalize an angle to (-pi, pi].
double normalizeAngle(double angle_rad);

// Normalize an angle to [0, 2*pi).
double normalizeAnglePositive(double angle_rad);

// Shortest signed difference a - b, wrapped to (-pi, pi].
double angleDifference(double a, double b);

// printf-style formatting helper.
std::string formatString(const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

// Monotonic wall clock in seconds (for logging).
double nowSeconds();

// Make sure the parent directory of `path` exists (recursively). Returns false
// on failure.
bool ensureParentDirectory(const std::string& path);

// True if the string looks like an integer index (e.g. "0", "2").
bool isIntegerIndex(const std::string& text);

// Simple scoped timer, milliseconds resolution.
class ScopedTimer {
 public:
  ScopedTimer() : start_(std::chrono::steady_clock::now()) {}
  void reset() { start_ = std::chrono::steady_clock::now(); }
  double elapsedMs() const {
    const auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(now - start_).count();
  }

 private:
  std::chrono::steady_clock::time_point start_;
};

}  // namespace glowtrack
