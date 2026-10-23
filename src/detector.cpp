// Green glowing target detector -- implementation.
//
// Target anatomy (see docs/recon_target_analysis.md): a saturated-white disc in
// the middle of a green glowing diffuser. The white disc boundary is the only
// clean, closed, roughly-circular edge in the scene, so it is the geometric
// primitive used by PnP. Two complementary extraction paths are used:
//
//   1. "hole" path (primary): the white disc appears as a HOLE in the green glow
//      connected component. With RETR_CCOMP contours the child (hole) contour is
//      already a clean closed boundary -> cv::fitEllipse gives the disc circle.
//      Validated by the white brightness of the interior + the green component
//      that surrounds it.
//   2. "white fallback" path: threshold the saturated-white pixels directly,
//      fit an ellipse to each round blob and validate it with a green annulus
//      around the ellipse. This keeps working when motion blur breaks the green
//      ring (the primary path then has no closed hole to fit).
//
// Both paths produce scored candidates; the highest score above
// `score_threshold` wins. Everything is computed inside the search ROI and then
// translated back to full-image coordinates.

#include "glowtrack/detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace glowtrack {
namespace {

constexpr double kPiD = 3.14159265358979323846;
constexpr float kPiF = static_cast<float>(kPiD);
constexpr float kDegToRadF = kPiF / 180.f;

// ---------------------------------------------------------------------------
// Target-anatomy gates. These are properties of the physical target, not of a
// deployment, so they are not part of DetectionConfig.
// ---------------------------------------------------------------------------
// Fraction of disc-interior samples that must look "white" (V >= white_v_min &&
// S <= white_s_max). Motion blur bleeds green into the disc edge, so this is
// deliberately not 1.0.
constexpr float kWhiteFracMinHole = 0.50f;
constexpr float kWhiteFracMinFallback = 0.55f;
// The disc is overexposed: most of its interior is near-blown-out white. This
// is the strongest soft-background discriminator (lab "white" objects are a
// couple of stops darker than the glowing disc) and is used as a hard gate plus
// a scoring term.
constexpr float kWhiteCoreVOffset = 30.f;
constexpr float kWhiteCoreSMax = 30.f;
constexpr float kWhiteCoreFracMin = 0.20f;
// The green glow must be bigger than the white disc.
constexpr float kGreenToDiscAreaMin = 1.5f;
// A real disc / hole is a compact blob: contour area close to the fitted
// ellipse area (rejects weird merged / snake-like blobs).
constexpr float kHoleFillMin = 0.75f;
constexpr float kWhiteFillMin = 0.60f;
// Candidate-count caps (speed + avoids pathological masks).
constexpr int kMaxHoleCandidates = 64;
constexpr int kMaxFallbackCandidates = 80;
// The hole path uses the true green-to-white boundary, so prefer it when both
// paths find the same target.
constexpr float kHolePathBonus = 0.04f;
// Minimum number of interior samples that must land inside the image.
constexpr int kMinWhiteSamples = 12;
// Number of ring radii sampled between ring_inner_scale and ring_outer_scale.
constexpr int kRingRadii = 3;

inline float clamp01(float v) {
  return v < 0.f ? 0.f : (v > 1.f ? 1.f : v);
}

inline float normalizeTo01(float value, float lo, float hi) {
  if (!(hi > lo)) {
    return value >= hi ? 1.f : 0.f;
  }
  return clamp01((value - lo) / (hi - lo));
}

// OpenCV morphology with the default anchor needs odd kernel sizes.
inline int oddKernelSize(int ksize) {
  if (ksize <= 1) {
    return 0;
  }
  return (ksize % 2 == 0) ? ksize + 1 : ksize;
}

// cv::RotatedRect (from cv::fitEllipse) -> Ellipse2D with semi_major >=
// semi_minor and angle_deg = direction of the major axis.
Ellipse2D toEllipse2D(const cv::RotatedRect& rect) {
  Ellipse2D ellipse;
  ellipse.center = rect.center;
  const bool width_is_major = rect.size.width >= rect.size.height;
  const float major =
      width_is_major ? rect.size.width : rect.size.height;
  const float minor =
      width_is_major ? rect.size.height : rect.size.width;
  ellipse.semi_major = 0.5f * major;
  ellipse.semi_minor = 0.5f * minor;
  float angle = rect.angle;
  if (!width_is_major) {
    angle += 90.f;
  }
  angle = std::fmod(angle, 180.f);
  if (angle < 0.f) {
    angle += 180.f;
  }
  ellipse.angle_deg = angle;
  return ellipse;
}

struct WhiteStats {
  float mean_v = 0.f;
  float mean_s = 0.f;
  float white_frac = 0.f;  // V >= white_v_min && S <= white_s_max
  float core_frac = 0.f;   // overexposed white: V >= white_v_min + 30 && S <= 30
  int samples = 0;
};

// Clamps a rectangle to the image; an empty rectangle means "whole image".
cv::Rect clampRect(const cv::Rect& rect, const cv::Size& size) {
  if (rect.width <= 0 || rect.height <= 0) {
    return cv::Rect(0, 0, size.width, size.height);
  }
  return rect & cv::Rect(0, 0, size.width, size.height);
}

}  // namespace

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------
struct TargetDetector::Impl {
  struct Candidate {
    Ellipse2D disc;
    cv::Rect2f bbox;
    float green_area = 0.f;
    float disc_area = 0.f;
    float axis_ratio = 0.f;
    float ring_ratio = 0.f;
    float fill_ratio = 0.f;
    float score = 0.f;
    bool from_hole = false;
    WhiteStats white;
    // Index of the enclosing top-level green contour (hole path) or -1.
    int parent_contour = -1;
    // Green contour used as the "glow" for fallback candidates (debug only).
    int glow_contour = -1;
  };

  // A top-level green connected component (the glow / background green blob).
  struct GreenComponent {
    int contour_index = -1;
    float area = 0.f;
    cv::Rect bbox;
  };

  DetectionConfig config;
  TargetGeometry target;

  cv::Mat open_kernel;
  cv::Mat close_kernel;
  std::vector<cv::Point2f> interior_offsets;  // normalised disc interior samples
  std::vector<cv::Point2f> ring_offsets;      // normalised annulus samples

  // Scratch buffers reused between frames (avoid per-frame allocations).
  cv::Mat hsv_work;
  cv::Mat green_mask;
  cv::Mat green_mask_hi;  // only used when the hue range wraps around 180
  cv::Mat white_mask;

  // Debug outputs (full-image coordinates / full image size).
  cv::Mat debug_mask;
  cv::Mat debug_candidates;
  std::vector<CandidateInfo> candidate_info;
  bool debug_enabled = true;

  Impl(const DetectionConfig& detector_config, const TargetGeometry& geometry)
      : config(detector_config), target(geometry) {
    buildKernels();
    buildSampleOffsets();
  }

  void buildKernels() {
    const int open_ksize = oddKernelSize(config.morph_open_ksize);
    const int close_ksize = oddKernelSize(config.morph_close_ksize);
    open_kernel = open_ksize > 0
                      ? cv::getStructuringElement(cv::MORPH_RECT,
                                                  cv::Size(open_ksize,
                                                           open_ksize))
                      : cv::Mat();
    close_kernel = close_ksize > 0
                       ? cv::getStructuringElement(cv::MORPH_RECT,
                                                   cv::Size(close_ksize,
                                                            close_ksize))
                       : cv::Mat();
  }

  void buildSampleOffsets() {
    // Disc interior: regular grid restricted to a disc of radius 0.8 (keeps the
    // samples away from the blurred boundary and the colour fringe).
    interior_offsets.clear();
    constexpr int kGrid = 9;
    for (int iy = 0; iy < kGrid; ++iy) {
      for (int ix = 0; ix < kGrid; ++ix) {
        const float u = 1.6f * ix / (kGrid - 1) - 0.8f;
        const float v = 1.6f * iy / (kGrid - 1) - 0.8f;
        if (u * u + v * v <= 1.0f) {
          interior_offsets.emplace_back(u, v);
        }
      }
    }
    // Annulus samples: kRingRadii radii x ring_samples angles.
    ring_offsets.clear();
    const int samples = std::max(8, config.ring_samples);
    const float inner = config.ring_inner_scale;
    const float outer = config.ring_outer_scale;
    for (int ri = 0; ri < kRingRadii; ++ri) {
      const float t = static_cast<float>(ri) /
                      static_cast<float>(std::max(1, kRingRadii - 1));
      const float scale = inner + t * (outer - inner);
      for (int si = 0; si < samples; ++si) {
        const float angle =
            2.f * kPiF * static_cast<float>(si) / static_cast<float>(samples);
        ring_offsets.emplace_back(scale * std::cos(angle),
                                  scale * std::sin(angle));
      }
    }
  }

  // Samples the HSV image inside the disc ellipse -> brightness/whiteness.
  WhiteStats sampleWhite(const cv::Mat& hsv, const Ellipse2D& ellipse) const {
    WhiteStats stats;
    const float ca = std::cos(ellipse.angle_deg * kDegToRadF);
    const float sa = std::sin(ellipse.angle_deg * kDegToRadF);
    const float v_min = static_cast<float>(config.white_v_min);
    const float s_max = static_cast<float>(config.white_s_max);
    const float v_core = std::max(v_min + kWhiteCoreVOffset, 230.f);
    const float s_core = std::max(0.5f * s_max, kWhiteCoreSMax);
    const int rows = hsv.rows;
    const int cols = hsv.cols;
    float sum_v = 0.f;
    float sum_s = 0.f;
    int white_count = 0;
    int core_count = 0;
    int count = 0;
    for (const cv::Point2f& offset : interior_offsets) {
      const int x = cvRound(ellipse.center.x +
                            ellipse.semi_major * offset.x * ca -
                            ellipse.semi_minor * offset.y * sa);
      const int y = cvRound(ellipse.center.y +
                            ellipse.semi_major * offset.x * sa +
                            ellipse.semi_minor * offset.y * ca);
      if (x < 0 || y < 0 || x >= cols || y >= rows) {
        continue;
      }
      const cv::Vec3b& pixel = hsv.at<cv::Vec3b>(y, x);
      const float v = static_cast<float>(pixel[2]);
      const float s = static_cast<float>(pixel[1]);
      sum_v += v;
      sum_s += s;
      ++count;
      if (v >= v_min && s <= s_max) {
        ++white_count;
      }
      if (v >= v_core && s <= s_core) {
        ++core_count;
      }
    }
    stats.samples = count;
    if (count > 0) {
      const float inv = 1.f / static_cast<float>(count);
      stats.mean_v = sum_v * inv;
      stats.mean_s = sum_s * inv;
      stats.white_frac = static_cast<float>(white_count) * inv;
      stats.core_frac = static_cast<float>(core_count) * inv;
    }
    return stats;
  }

  // Fraction of the annulus samples (around the disc) that fall on green pixels.
  float sampleRingRatio(const cv::Mat& green, const Ellipse2D& ellipse) const {
    const float ca = std::cos(ellipse.angle_deg * kDegToRadF);
    const float sa = std::sin(ellipse.angle_deg * kDegToRadF);
    const int rows = green.rows;
    const int cols = green.cols;
    int green_count = 0;
    int total = 0;
    for (const cv::Point2f& offset : ring_offsets) {
      const int x = cvRound(ellipse.center.x +
                            ellipse.semi_major * offset.x * ca -
                            ellipse.semi_minor * offset.y * sa);
      const int y = cvRound(ellipse.center.y +
                            ellipse.semi_major * offset.x * sa +
                            ellipse.semi_minor * offset.y * ca);
      if (x < 0 || y < 0 || x >= cols || y >= rows) {
        continue;
      }
      ++total;
      if (green.at<unsigned char>(y, x) != 0) {
        ++green_count;
      }
    }
    return total > 0 ? static_cast<float>(green_count) /
                           static_cast<float>(total)
                     : 0.f;
  }

  // Validates one ellipse candidate and computes its score. `contour_area` is
  // the area of the contour the ellipse was fitted to (fill-ratio sanity),
  // `green_area` the area of the enclosing green component (hole path only).
  bool evaluate(const cv::Mat& hsv, const cv::Mat& green,
                const Ellipse2D& disc, float contour_area, bool from_hole,
                float green_area, Candidate* candidate) const {
    if (!disc.valid()) {
      return false;
    }
    const float axis_ratio = disc.semi_minor / disc.semi_major;
    if (axis_ratio < config.min_axis_ratio ||
        axis_ratio > config.max_axis_ratio) {
      return false;
    }
    const float disc_area = kPiF * disc.semi_major * disc.semi_minor;
    if (disc_area < static_cast<float>(config.min_area_px) ||
        disc_area > static_cast<float>(config.max_area_px)) {
      return false;
    }
    if (disc.center.x < 0.f || disc.center.y < 0.f ||
        disc.center.x >= static_cast<float>(hsv.cols) ||
        disc.center.y >= static_cast<float>(hsv.rows)) {
      return false;
    }
    // Compactness: a disc contour should mostly fill its fitted ellipse.
    const float fill_ratio = disc_area > 0.f ? contour_area / disc_area : 0.f;
    if (contour_area > 0.f &&
        fill_ratio < (from_hole ? kHoleFillMin : kWhiteFillMin)) {
      return false;
    }
    if (from_hole) {
      // The green glow must extend well beyond the white disc.
      if (green_area < kGreenToDiscAreaMin * disc_area) {
        return false;
      }
    }
    const WhiteStats white = sampleWhite(hsv, disc);
    if (white.samples < kMinWhiteSamples) {
      return false;
    }
    if (white.mean_v < static_cast<float>(config.white_v_min)) {
      return false;
    }
    const float white_frac_min =
        from_hole ? kWhiteFracMinHole : kWhiteFracMinFallback;
    if (white.white_frac < white_frac_min) {
      return false;
    }
    // Reject dim "white" lab objects (paper, lamps, reflections): the target
    // disc is a self-illuminated, blown-out white area.
    if (white.core_frac < kWhiteCoreFracMin) {
      return false;
    }
    const float ring_ratio = sampleRingRatio(green, disc);
    if (ring_ratio < config.ring_green_min_ratio) {
      return false;
    }

    // ---- score in [0, 1] ---------------------------------------------------
    const float brightness =
        normalizeTo01(white.mean_v, static_cast<float>(config.white_v_min),
                      255.f);
    const float whiteness = normalizeTo01(white.white_frac, 0.5f, 1.f);
    const float core = normalizeTo01(white.core_frac, 0.2f, 0.95f);
    const float low_saturation =
        1.f - normalizeTo01(white.mean_s, 0.f,
                            3.f * static_cast<float>(config.white_s_max));
    const float white_score = 0.35f * brightness + 0.20f * whiteness +
                              0.30f * core + 0.15f * low_saturation;
    const float ring_score =
        normalizeTo01(ring_ratio, config.ring_green_min_ratio, 1.f);
    const float shape_score =
        normalizeTo01(axis_ratio, config.min_axis_ratio, 1.f);
    float glow_score = ring_score;
    if (from_hole && green_area > 0.f) {
      glow_score = normalizeTo01(green_area / disc_area,
                                 kGreenToDiscAreaMin, 10.f);
    }
    float score = 0.30f * white_score + 0.25f * ring_score +
                  0.20f * shape_score + 0.25f * glow_score;
    if (from_hole) {
      score += kHolePathBonus;
    }

    candidate->disc = disc;
    candidate->disc_area = disc_area;
    candidate->axis_ratio = axis_ratio;
    candidate->ring_ratio = ring_ratio;
    candidate->fill_ratio = fill_ratio;
    candidate->white = white;
    candidate->score = clamp01(score);
    candidate->from_hole = from_hole;
    candidate->green_area = green_area;
    return true;
  }

  // Primary path: holes in green connected components.
  void collectHoleCandidates(
      const cv::Mat& hsv, const cv::Mat& green,
      const std::vector<std::vector<cv::Point>>& contours,
      const std::vector<cv::Vec4i>& hierarchy,
      const std::vector<GreenComponent>& components,
      std::vector<Candidate>* candidates) const {
    // Map top-level contour index -> component info.
    std::vector<int> component_of_contour(contours.size(), -1);
    for (size_t ci = 0; ci < components.size(); ++ci) {
      component_of_contour[static_cast<size_t>(components[ci].contour_index)] =
          static_cast<int>(ci);
    }
    const float min_hole_contour_area =
        0.5f * static_cast<float>(config.min_area_px);
    for (size_t i = 0; i < contours.size(); ++i) {
      const int parent = hierarchy[i][3];
      if (parent < 0) {
        continue;  // not a hole
      }
      if (contours[i].size() < 5) {
        continue;
      }
      const int component = component_of_contour[static_cast<size_t>(parent)];
      if (component < 0) {
        continue;
      }
      const float parent_area = components[static_cast<size_t>(component)].area;
      if (parent_area < static_cast<float>(config.min_area_px)) {
        continue;
      }
      const float hole_contour_area =
          static_cast<float>(cv::contourArea(contours[i]));
      if (hole_contour_area < min_hole_contour_area) {
        continue;
      }
      const Ellipse2D disc = toEllipse2D(cv::fitEllipse(contours[i]));
      Candidate candidate;
      if (!evaluate(hsv, green, disc, hole_contour_area, /*from_hole=*/true,
                    parent_area, &candidate)) {
        continue;
      }
      candidate.parent_contour = parent;
      candidate.bbox = cv::Rect2f(components[static_cast<size_t>(component)]
                                      .bbox);
      candidates->push_back(candidate);
      if (static_cast<int>(candidates->size()) >= kMaxHoleCandidates) {
        break;
      }
    }
  }

  // Fallback path: saturated-white blobs validated by a green annulus.
  void collectFallbackCandidates(
      const cv::Mat& hsv, const cv::Mat& green,
      const std::vector<std::vector<cv::Point>>& white_contours,
      const std::vector<GreenComponent>& components,
      std::vector<Candidate>* candidates) const {
    // Evaluate the biggest white blobs first: the disc is a large overexposed
    // region, small white speckles cannot be it (cheap pre-filter).
    struct Blob {
      float area;
      size_t index;
    };
    std::vector<Blob> blobs;
    blobs.reserve(white_contours.size());
    const float min_area = static_cast<float>(config.min_area_px);
    const float max_area = static_cast<float>(config.max_area_px);
    for (size_t i = 0; i < white_contours.size(); ++i) {
      if (white_contours[i].size() < 5) {
        continue;
      }
      const float area =
          static_cast<float>(cv::contourArea(white_contours[i]));
      if (area < min_area || area > max_area) {
        continue;
      }
      blobs.push_back({area, i});
    }
    std::sort(blobs.begin(), blobs.end(),
              [](const Blob& a, const Blob& b) { return a.area > b.area; });
    if (static_cast<int>(blobs.size()) > kMaxFallbackCandidates) {
      blobs.resize(static_cast<size_t>(kMaxFallbackCandidates));
    }
    for (const Blob& blob : blobs) {
      const std::vector<cv::Point>& contour = white_contours[blob.index];
      Candidate candidate;
      const Ellipse2D disc = toEllipse2D(cv::fitEllipse(contour));
      if (!evaluate(hsv, green, disc, blob.area, /*from_hole=*/false, 0.f,
                    &candidate)) {
        continue;
      }
      candidate.parent_contour = -1;
      candidates->push_back(candidate);
    }
  }

  // Attaches the enclosing green component (bbox / area) to candidates that do
  // not come from the hole path. Used for Detection::bbox / area_px / debug.
  // Must be called before the candidates are translated to full-image
  // coordinates (component boxes are ROI-local).
  void resolveGlowComponents(const std::vector<GreenComponent>& components,
                             std::vector<Candidate>* candidates) const {
    for (Candidate& candidate : *candidates) {
      if (candidate.parent_contour >= 0 || candidate.green_area > 0.f) {
        continue;
      }
      // The glowing ring around the disc is the green component whose bounding
      // box surrounds the disc centre most tightly.
      const cv::Point2f center = candidate.disc.center;
      float best_d2 = FLT_MAX;
      const GreenComponent* best = nullptr;
      for (const GreenComponent& component : components) {
        const cv::Rect& bbox = component.bbox;
        const bool contains =
            center.x >= bbox.x && center.x < bbox.x + bbox.width &&
            center.y >= bbox.y && center.y < bbox.y + bbox.height;
        if (!contains) {
          continue;
        }
        if (component.area < kGreenToDiscAreaMin * candidate.disc_area) {
          continue;
        }
        const float cx = bbox.x + 0.5f * bbox.width;
        const float cy = bbox.y + 0.5f * bbox.height;
        const float d2 = (cx - center.x) * (cx - center.x) +
                         (cy - center.y) * (cy - center.y);
        if (d2 < best_d2) {
          best_d2 = d2;
          best = &component;
        }
      }
      if (best != nullptr) {
        candidate.green_area = best->area;
        candidate.bbox = cv::Rect2f(best->bbox);
        candidate.glow_contour = best->contour_index;
      } else {
        // No green component covers the disc: keep a plausible box for debug.
        const float scale = config.ring_outer_scale;
        candidate.green_area = candidate.disc_area;
        candidate.bbox = cv::Rect2f(
            center.x - scale * candidate.disc.semi_major,
            center.y - scale * candidate.disc.semi_minor,
            2.f * scale * candidate.disc.semi_major,
            2.f * scale * candidate.disc.semi_minor);
      }
    }
  }

  void renderDebug(const cv::Mat& bgr, const cv::Rect& roi,
                   const std::vector<Candidate>& candidates,
                   int best_index) {
    const cv::Size size = bgr.size();
    debug_mask = cv::Mat::zeros(size, CV_8UC3);
    cv::Mat mask_view = debug_mask(roi);
    mask_view.setTo(cv::Scalar(0, 190, 0), green_mask);
    if (!white_mask.empty()) {
      mask_view.setTo(cv::Scalar(235, 235, 235), white_mask);
    }
    cv::rectangle(debug_mask, roi, cv::Scalar(255, 90, 0), 1);

    debug_candidates = bgr.clone();
    cv::rectangle(debug_candidates, roi, cv::Scalar(255, 90, 0), 1);
    for (size_t i = 0; i < candidates.size(); ++i) {
      const Candidate& candidate = candidates[i];
      const bool accepted = candidate.score >= config.score_threshold;
      cv::Scalar color = accepted ? cv::Scalar(255, 160, 0)      // blue-ish
                                  : cv::Scalar(0, 140, 255);     // orange
      int thickness = 1;
      if (static_cast<int>(i) == best_index) {
        color = cv::Scalar(0, 0, 255);
        thickness = 2;
      }
      const cv::Point center(cvRound(candidate.disc.center.x),
                             cvRound(candidate.disc.center.y));
      cv::ellipse(debug_candidates, center,
                  cv::Size2f(candidate.disc.semi_major,
                             candidate.disc.semi_minor),
                  candidate.disc.angle_deg, 0.0, 360.0, color, thickness);
      cv::circle(debug_candidates, center, 2, color, -1);
      char text[48];
      std::snprintf(text, sizeof(text), "%.2f%s", candidate.score,
                    candidate.from_hole ? "H" : "W");
      cv::putText(debug_candidates, text, center + cv::Point(4, -6),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv::LINE_AA);
    }
  }

  Detection detect(const cv::Mat& bgr, const cv::Rect& search_roi,
                   int frame_index) {
    Detection detection;
    detection.frame_index = frame_index;
    candidate_info.clear();
    debug_mask = cv::Mat();
    debug_candidates = cv::Mat();
    if (bgr.empty()) {
      return detection;
    }

    // Normalise the input to 8-bit BGR (the pipeline feeds BGR, but accepting
    // grayscale / BGRA keeps the unit tests and offline tools simple).
    cv::Mat bgr_full = bgr;
    if (bgr.type() == CV_8UC4) {
      cv::cvtColor(bgr, bgr_bgra_work, cv::COLOR_BGRA2BGR);
      bgr_full = bgr_bgra_work;
    } else if (bgr.type() == CV_8UC1) {
      cv::cvtColor(bgr, bgr_gray_work, cv::COLOR_GRAY2BGR);
      bgr_full = bgr_gray_work;
    } else if (bgr.type() != CV_8UC3) {
      return detection;
    }

    const cv::Size image_size = bgr_full.size();
    cv::Rect roi = clampRect(search_roi, image_size);
    if (roi.width < 8 || roi.height < 8) {
      return detection;
    }
    // Grow the processing ROI a little so the annulus samples around a disc
    // that sits at the ROI border are not clipped by the crop.
    {
      const int margin = std::max(8, std::min(roi.width, roi.height) / 10);
      roi = clampRect(cv::Rect(roi.x - margin, roi.y - margin,
                               roi.width + 2 * margin,
                               roi.height + 2 * margin),
                      image_size);
    }
    const cv::Mat view = bgr_full(roi);

    cv::cvtColor(view, hsv_work, cv::COLOR_BGR2HSV);

    // ---- green glow mask -----------------------------------------------
    const cv::Scalar green_lo(config.green_h_min, config.green_s_min,
                              config.green_v_min);
    const cv::Scalar green_hi(config.green_h_max, 255, 255);
    if (config.green_h_min <= config.green_h_max) {
      cv::inRange(hsv_work, green_lo, green_hi, green_mask);
    } else {
      // Hue range wraps around 180 (rare, but supported).
      cv::inRange(hsv_work, cv::Scalar(0, config.green_s_min,
                                       config.green_v_min),
                  cv::Scalar(config.green_h_max, 255, 255), green_mask);
      cv::inRange(hsv_work,
                  cv::Scalar(config.green_h_min, config.green_s_min,
                             config.green_v_min),
                  cv::Scalar(180, 255, 255), green_mask_hi);
      cv::bitwise_or(green_mask, green_mask_hi, green_mask);
    }
    if (!open_kernel.empty()) {
      cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN, open_kernel);
    }
    if (!close_kernel.empty()) {
      cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, close_kernel);
    }

    std::vector<std::vector<cv::Point>> green_contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(green_mask, green_contours, hierarchy, cv::RETR_CCOMP,
                     cv::CHAIN_APPROX_SIMPLE);

    // Index the top-level green connected components.
    std::vector<GreenComponent> components;
    if (!hierarchy.empty()) {
      components.reserve(green_contours.size());
      for (size_t i = 0; i < green_contours.size(); ++i) {
        if (hierarchy[i][3] >= 0) {
          continue;  // hole, not a connected component
        }
        GreenComponent component;
        component.contour_index = static_cast<int>(i);
        component.area = static_cast<float>(cv::contourArea(green_contours[i]));
        component.bbox = cv::boundingRect(green_contours[i]);
        if (component.area < static_cast<float>(config.min_area_px)) {
          continue;
        }
        components.push_back(component);
      }
    }

    std::vector<Candidate> candidates;
    candidates.reserve(8);
    if (!components.empty() && !hierarchy.empty()) {
      collectHoleCandidates(hsv_work, green_mask, green_contours, hierarchy,
                            components, &candidates);
    }

    // ---- saturated-white fallback path ----------------------------------
    if (config.white_fallback_enabled) {
      cv::inRange(hsv_work,
                  cv::Scalar(0, 0, config.white_v_min),
                  cv::Scalar(180, config.white_s_max, 255), white_mask);
      std::vector<std::vector<cv::Point>> white_contours;
      cv::findContours(white_mask, white_contours, cv::RETR_EXTERNAL,
                       cv::CHAIN_APPROX_SIMPLE);
      collectFallbackCandidates(hsv_work, green_mask, white_contours,
                                components, &candidates);
    } else {
      white_mask = cv::Mat();
    }

    // ---- select the best candidate --------------------------------------
    // Attach glow bbox/area to fallback candidates while the component boxes
    // are still in ROI-local coordinates.
    resolveGlowComponents(components, &candidates);

    int best_index = -1;
    float best_score = -1.f;
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (candidates[i].score > best_score) {
        best_score = candidates[i].score;
        best_index = static_cast<int>(i);
      }
    }

    // ---- translate back to full-image coordinates ------------------------
    const float offset_x = static_cast<float>(roi.x);
    const float offset_y = static_cast<float>(roi.y);
    for (Candidate& candidate : candidates) {
      candidate.disc.center.x += offset_x;
      candidate.disc.center.y += offset_y;
      candidate.bbox.x += offset_x;
      candidate.bbox.y += offset_y;
    }

    if (best_index >= 0 && best_score >= config.score_threshold) {
      const Candidate& best = candidates[static_cast<size_t>(best_index)];
      detection.valid = true;
      detection.disc_ellipse = best.disc;
      detection.score = best.score;
      detection.bbox = best.bbox;
      detection.area_px = best.green_area;
      int glow_contour = best.parent_contour >= 0 ? best.parent_contour
                                                  : best.glow_contour;
      if (glow_contour >= 0 &&
          glow_contour < static_cast<int>(green_contours.size()) &&
          green_contours[static_cast<size_t>(glow_contour)].size() >= 5) {
        Ellipse2D glow = toEllipse2D(cv::fitEllipse(
            green_contours[static_cast<size_t>(glow_contour)]));
        glow.center.x += offset_x;
        glow.center.y += offset_y;
        detection.glow_ellipse = glow;
      } else {
        detection.glow_ellipse = detection.disc_ellipse;
      }
    }

    // ---- debug outputs ---------------------------------------------------
    candidate_info.clear();
    candidate_info.reserve(candidates.size());
    for (const Candidate& candidate : candidates) {
      CandidateInfo info;
      info.disc = candidate.disc;
      info.score = candidate.score;
      info.accepted = candidate.score >= config.score_threshold;
      info.from_hole = candidate.from_hole;
      info.ring_ratio = candidate.ring_ratio;
      info.white_frac = candidate.white.white_frac;
      info.core_frac = candidate.white.core_frac;
      info.fill_ratio = candidate.fill_ratio;
      info.mean_v = candidate.white.mean_v;
      info.mean_s = candidate.white.mean_s;
      info.glow_area = candidate.green_area;
      candidate_info.push_back(info);
    }
    if (debug_enabled) {
      renderDebug(bgr_full, roi, candidates, best_index);
    }
    return detection;
  }

  cv::Mat bgr_bgra_work;
  cv::Mat bgr_gray_work;
};

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------
TargetDetector::TargetDetector(const DetectionConfig& detector_config,
                               const TargetGeometry& target)
    : impl_(new Impl(detector_config, target)) {}

TargetDetector::~TargetDetector() = default;

Detection TargetDetector::detect(const cv::Mat& bgr,
                                 const cv::Rect& search_roi,
                                 int frame_index) {
  return impl_->detect(bgr, search_roi, frame_index);
}

void TargetDetector::reset() {
  impl_->candidate_info.clear();
  impl_->debug_mask = cv::Mat();
  impl_->debug_candidates = cv::Mat();
}

const cv::Mat& TargetDetector::debugMask() const { return impl_->debug_mask; }

const cv::Mat& TargetDetector::debugCandidates() const {
  return impl_->debug_candidates;
}

const std::vector<TargetDetector::CandidateInfo>&
TargetDetector::debugCandidateInfo() const {
  return impl_->candidate_info;
}

void TargetDetector::setDebugEnabled(bool enabled) {
  impl_->debug_enabled = enabled;
}

bool TargetDetector::debugEnabled() const { return impl_->debug_enabled; }

}  // namespace glowtrack
