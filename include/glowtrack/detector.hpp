#pragma once
// Green glowing target detector.
//
// Pipeline
//   BGR image
//     -> HSV green-glow mask (morphological cleaning)
//     -> connected green regions + holes (the white disc is a hole)
//     -> ellipse fit on the white disc boundary (geometric PnP primitive)
//     -> validation: bright/saturated disc, green annulus around it,
//        plausible size & circularity
//     -> candidate scoring, best candidate wins (optionally inside an ROI).

#include "glowtrack/config.hpp"
#include "glowtrack/types.hpp"

#include <opencv2/core.hpp>

#include <memory>

namespace glowtrack {

class TargetDetector {
 public:
  TargetDetector(const DetectionConfig& detector_config,
                 const TargetGeometry& target);
  ~TargetDetector();

  TargetDetector(const TargetDetector&) = delete;
  TargetDetector& operator=(const TargetDetector&) = delete;

  // Runs detection. If search_roi is non-empty the detector only looks inside
  // it (coordinates are relative to the full image). Returns a Detection whose
  // .valid is false when no candidate passes the filters.
  Detection detect(const cv::Mat& bgr, const cv::Rect& search_roi,
                   int frame_index);

  Detection detect(const cv::Mat& bgr, int frame_index) {
    return detect(bgr, cv::Rect(), frame_index);
  }

  // Resets temporal/history state (not required to be called between videos).
  void reset();

  // Debug views (valid after detect()):
  const cv::Mat& debugMask() const;
  const cv::Mat& debugCandidates() const;

  struct Impl;  // pimpl keeps the public interface stable while tuning

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace glowtrack
