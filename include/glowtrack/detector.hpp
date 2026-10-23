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
#include <vector>

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
  //   debugMask()       : BGR view of the segmentation (green = green mask,
  //                       white = saturated-white mask, blue rect = processed
  //                       ROI). Empty when the last input image was invalid.
  //   debugCandidates() : copy of the input image with every evaluated
  //                       candidate drawn (blue = accepted, orange = rejected,
  //                       red = winning candidate).
  const cv::Mat& debugMask() const;
  const cv::Mat& debugCandidates() const;

  // Detailed candidate list of the last detect() (full-image coordinates).
  // Additive tuning helper, does not affect the tracking pipeline.
  struct CandidateInfo {
    Ellipse2D disc;
    float score = 0.f;
    bool accepted = false;
    bool from_hole = false;
    float ring_ratio = 0.f;
    float white_frac = 0.f;
    float core_frac = 0.f;  // overexposed (blown-out) white fraction
    float fill_ratio = 0.f; // contour area / fitted ellipse area
    float mean_v = 0.f;
    float mean_s = 0.f;
    float glow_area = 0.f;
  };
  const std::vector<CandidateInfo>& debugCandidateInfo() const;

  // Debug image construction can be disabled for production / timing runs
  // (it costs ~0.5-1 ms per large frame). Default: enabled.
  void setDebugEnabled(bool enabled);
  bool debugEnabled() const;

  struct Impl;  // pimpl keeps the public interface stable while tuning

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace glowtrack
