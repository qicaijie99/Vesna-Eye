// Unit tests for the green glow target detector (src/detector.cpp).
//
// The synthetic scenes mimic the real target anatomy from
// docs/recon_target_analysis.md: a saturated-white disc surrounded by a green
// glowing ring / diffuser. They are rendered with cv::ellipse + a slight blur,
// which reproduces the soft (JPEG / defocus) edges of the real camera frames.

#include "test_framework.hpp"

#include "glowtrack/detector.hpp"
#include "glowtrack/types.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <vector>

namespace {

using glowtrack::Detection;
using glowtrack::DetectionConfig;
using glowtrack::TargetDetector;
using glowtrack::TargetGeometry;

struct TargetSpec {
  cv::Point2f center{320.f, 240.f};
  float radius = 30.f;    // semi-major axis of the white disc [px]
  float axis_ratio = 1.f; // semi-minor / semi-major
  float angle_deg = 0.f;  // orientation of the major axis
  float glow_scale = 1.9f;  // outer green radius / disc semi-major
};

// Renders the target on a dark lab-like background. `draw_glow` / `draw_disc`
// allow rendering "green only" / "white only" control scenes.
cv::Mat renderTargetScene(const cv::Size& size, const TargetSpec& spec,
                          bool draw_glow = true, bool draw_disc = true) {
  cv::Mat image(size, CV_8UC3, cv::Scalar(46, 48, 50));
  const float minor = spec.radius * spec.axis_ratio;
  const cv::Point center(cvRound(spec.center.x), cvRound(spec.center.y));
  if (draw_glow) {
    cv::ellipse(image, center,
                cv::Size(cvRound(spec.radius * spec.glow_scale),
                         cvRound(minor * spec.glow_scale)),
                spec.angle_deg, 0.0, 360.0, cv::Scalar(18, 165, 22),
                cv::FILLED);
  }
  if (draw_disc) {
    cv::ellipse(image, center,
                cv::Size(cvRound(spec.radius), cvRound(minor)),
                spec.angle_deg, 0.0, 360.0, cv::Scalar(250, 250, 250),
                cv::FILLED);
  }
  // Mild sensor noise + soft edges (real frames are defocused / JPEG coded).
  cv::Mat noise(size, CV_8UC3);
  cv::randu(noise, cv::Scalar(0, 0, 0), cv::Scalar(5, 5, 5));
  image += noise;
  cv::GaussianBlur(image, image, cv::Size(3, 3), 0.6);
  return image;
}

TargetDetector makeDetector() {
  return TargetDetector(DetectionConfig(), TargetGeometry());
}

// Difference of two ellipse orientations modulo 180 degrees.
float angleDifference180(float a, float b) {
  float diff = std::fabs(a - b);
  while (diff > 90.f) {
    diff = std::fabs(diff - 180.f);
  }
  return diff;
}

void checkDetectionNear(const Detection& detection, const TargetSpec& spec,
                        float center_tolerance_px) {
  GT_CHECK(detection.valid);
  GT_CHECK_NEAR(detection.disc_ellipse.center.x, spec.center.x,
                center_tolerance_px);
  GT_CHECK_NEAR(detection.disc_ellipse.center.y, spec.center.y,
                center_tolerance_px);
  const float expected_major = spec.radius;
  const float expected_minor = spec.radius * spec.axis_ratio;
  GT_CHECK_NEAR(detection.disc_ellipse.semi_major, expected_major,
                0.10f * expected_major);
  GT_CHECK_NEAR(detection.disc_ellipse.semi_minor, expected_minor,
                0.10f * expected_major);
  // The fitted disc must be a real ellipse, not a degenerate one.
  GT_CHECK(detection.disc_ellipse.semi_major >=
           detection.disc_ellipse.semi_minor);
  GT_CHECK_GT(detection.score, 0.25f);
}

}  // namespace

// ---------------------------------------------------------------------------
// Detection accuracy at multiple positions / scales / tilts.
// ---------------------------------------------------------------------------
GT_TEST(detector_synthetic_target_positions_scales_tilts) {
  TargetDetector detector = makeDetector();
  const cv::Size size(640, 480);
  const std::vector<cv::Point2f> centers = {
      {320.f, 240.f}, {180.f, 150.f}, {470.f, 120.f}, {140.f, 350.f},
      {500.f, 370.f}};
  const std::vector<float> radii = {14.f, 22.f, 34.f};
  const std::vector<float> ratios = {1.0f, 0.85f, 0.68f};
  int frame_index = 0;
  for (const cv::Point2f& center : centers) {
    for (float radius : radii) {
      for (float ratio : ratios) {
        TargetSpec spec;
        spec.center = center;
        spec.radius = radius;
        spec.axis_ratio = ratio;
        spec.angle_deg = 20.f;
        const cv::Mat image = renderTargetScene(size, spec);
        const Detection detection =
            detector.detect(image, frame_index++);
        checkDetectionNear(detection, spec, 2.0f);
        GT_CHECK(detection.frame_index == frame_index - 1);
        GT_CHECK_NEAR(detection.disc_ellipse.semi_minor /
                          detection.disc_ellipse.semi_major,
                      ratio, 0.10f);
      }
    }
  }
}

// The major-axis orientation of a clearly elliptical disc must be recovered.
GT_TEST(detector_synthetic_tilt_orientation) {
  TargetDetector detector = makeDetector();
  const cv::Size size(640, 480);
  for (float angle : {0.f, 25.f, 60.f, 115.f}) {
    TargetSpec spec;
    spec.center = cv::Point2f(320.f, 240.f);
    spec.radius = 32.f;
    spec.axis_ratio = 0.65f;
    spec.angle_deg = angle;
    const cv::Mat image = renderTargetScene(size, spec);
    const Detection detection = detector.detect(image, 0);
    GT_CHECK(detection.valid);
    const float fitted_ratio = detection.disc_ellipse.semi_minor /
                               detection.disc_ellipse.semi_major;
    GT_CHECK_NEAR(fitted_ratio, 0.65f, 0.08f);
    GT_CHECK_LT(angleDifference180(detection.disc_ellipse.angle_deg, angle),
                12.f);
  }
}

// ---------------------------------------------------------------------------
// Rejection: blank image, green-only, white-only, white lamp, green object.
// ---------------------------------------------------------------------------
GT_TEST(detector_rejects_blank_image) {
  TargetDetector detector = makeDetector();
  const cv::Mat blank(480, 640, CV_8UC3, cv::Scalar(52, 54, 56));
  const Detection detection = detector.detect(blank, 3);
  GT_CHECK(!detection.valid);
  GT_CHECK(detection.frame_index == 3);
}

GT_TEST(detector_rejects_empty_image) {
  TargetDetector detector = makeDetector();
  const cv::Mat empty;
  const Detection detection = detector.detect(empty, 7);
  GT_CHECK(!detection.valid);
  GT_CHECK(detection.frame_index == 7);
  GT_CHECK(detector.debugMask().empty());
  GT_CHECK(detector.debugCandidates().empty());
}

GT_TEST(detector_rejects_green_only) {
  TargetDetector detector = makeDetector();
  TargetSpec spec;
  spec.center = cv::Point2f(320.f, 240.f);
  spec.radius = 30.f;
  // Green glow without the white disc -> no valid disc candidate.
  const cv::Mat green_only = renderTargetScene(cv::Size(640, 480), spec,
                                               /*draw_glow=*/true,
                                               /*draw_disc=*/false);
  GT_CHECK(!detector.detect(green_only, 0).valid);
}

GT_TEST(detector_rejects_white_only) {
  TargetDetector detector = makeDetector();
  TargetSpec spec;
  spec.center = cv::Point2f(320.f, 240.f);
  spec.radius = 28.f;
  // White disc without any green around it -> ring validation fails.
  const cv::Mat white_only = renderTargetScene(cv::Size(640, 480), spec,
                                               /*draw_glow=*/false,
                                               /*draw_disc=*/true);
  GT_CHECK(!detector.detect(white_only, 0).valid);
}

GT_TEST(detector_rejects_white_lamp_without_ring) {
  TargetDetector detector = makeDetector();
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(46, 48, 50));
  // A saturated white lamp panel (green glare is present elsewhere in the
  // scene, just like the lab background, but not around the lamp).
  cv::rectangle(image, cv::Rect(420, 60, 120, 70), cv::Scalar(252, 252, 252),
                cv::FILLED);
  cv::rectangle(image, cv::Rect(40, 340, 110, 90), cv::Scalar(20, 170, 25),
                cv::FILLED);
  cv::GaussianBlur(image, image, cv::Size(3, 3), 0.6);
  GT_CHECK(!detector.detect(image, 0).valid);
}

GT_TEST(detector_rejects_green_object_without_white_hole) {
  TargetDetector detector = makeDetector();
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(46, 48, 50));
  // Lab background green object: filled blob, no white disc inside.
  cv::ellipse(image, cv::Point(330, 250), cv::Size(90, 60), 12.0, 0.0, 360.0,
              cv::Scalar(18, 168, 22), cv::FILLED);
  cv::GaussianBlur(image, image, cv::Size(3, 3), 0.6);
  GT_CHECK(!detector.detect(image, 0).valid);
}

// ---------------------------------------------------------------------------
// Candidate selection with distractors.
// ---------------------------------------------------------------------------
GT_TEST(detector_prefers_target_over_distractors) {
  TargetDetector detector = makeDetector();
  const cv::Size size(640, 480);
  TargetSpec spec;
  spec.center = cv::Point2f(250.f, 260.f);
  spec.radius = 26.f;
  spec.axis_ratio = 0.9f;
  cv::Mat image = renderTargetScene(size, spec);
  // White distractor (lamp / overexposed paper) + a green background object.
  cv::rectangle(image, cv::Rect(480, 60, 110, 60), cv::Scalar(248, 248, 248),
                cv::FILLED);
  cv::rectangle(image, cv::Rect(30, 360, 130, 90), cv::Scalar(20, 160, 25),
                cv::FILLED);
  cv::GaussianBlur(image, image, cv::Size(3, 3), 0.6);
  const Detection detection = detector.detect(image, 0);
  checkDetectionNear(detection, spec, 2.0f);
}

// ---------------------------------------------------------------------------
// ROI search: same best candidate as the full-frame search, coordinates still
// absolute.
// ---------------------------------------------------------------------------
GT_TEST(detector_roi_matches_full_search) {
  TargetDetector detector = makeDetector();
  const cv::Size size(640, 480);
  for (const cv::Point2f& center :
       {cv::Point2f(320.f, 240.f), cv::Point2f(430.f, 300.f)}) {
    TargetSpec spec;
    spec.center = center;
    spec.radius = 27.f;
    spec.axis_ratio = 0.82f;
    spec.angle_deg = 30.f;
    const cv::Mat image = renderTargetScene(size, spec);

    const Detection full = detector.detect(image, 0);
    GT_CHECK(full.valid);

    const int half = static_cast<int>(3.0f * spec.radius);
    const cv::Rect roi =
        cv::Rect(cvRound(spec.center.x) - half, cvRound(spec.center.y) - half,
                 2 * half, 2 * half) &
        cv::Rect(0, 0, size.width, size.height);
    GT_CHECK(roi.width > 0 && roi.height > 0);
    const Detection roi_detection = detector.detect(image, roi, 1);
    GT_CHECK(roi_detection.valid);

    // Coordinates are returned in full-image space (compare to the target's
    // true position, not to the ROI-local one).
    GT_CHECK_NEAR(roi_detection.disc_ellipse.center.x, spec.center.x, 2.0f);
    GT_CHECK_NEAR(roi_detection.disc_ellipse.center.y, spec.center.y, 2.0f);
    // Same best candidate as the full-frame search.
    GT_CHECK_NEAR(roi_detection.disc_ellipse.center.x,
                  full.disc_ellipse.center.x, 1.0f);
    GT_CHECK_NEAR(roi_detection.disc_ellipse.center.y,
                  full.disc_ellipse.center.y, 1.0f);
    GT_CHECK_NEAR(roi_detection.disc_ellipse.semi_major,
                  full.disc_ellipse.semi_major, 0.02f * spec.radius + 0.01f);
    GT_CHECK_NEAR(roi_detection.disc_ellipse.semi_minor,
                  full.disc_ellipse.semi_minor, 0.02f * spec.radius + 0.01f);
    GT_CHECK_NEAR(roi_detection.area_px, full.area_px,
                  0.05f * std::max(1.f, full.area_px));
  }
}

GT_TEST(detector_roi_outside_target_finds_nothing) {
  TargetDetector detector = makeDetector();
  TargetSpec spec;
  spec.center = cv::Point2f(320.f, 240.f);
  spec.radius = 28.f;
  const cv::Mat image = renderTargetScene(cv::Size(640, 480), spec);
  // ROI far away from the target: no candidate inside -> invalid detection.
  const cv::Rect roi(0, 0, 120, 120);
  GT_CHECK(!detector.detect(image, roi, 0).valid);
}

// ---------------------------------------------------------------------------
// Score threshold behaviour (the only config value that gates the final pick).
// ---------------------------------------------------------------------------
GT_TEST(detector_score_threshold) {
  TargetSpec spec;
  spec.center = cv::Point2f(320.f, 240.f);
  spec.radius = 30.f;
  spec.axis_ratio = 0.85f;
  const cv::Mat image = renderTargetScene(cv::Size(640, 480), spec);

  DetectionConfig impossible_config;
  impossible_config.score_threshold = 1.01f;  // no candidate can reach 1.01
  TargetDetector strict_detector(impossible_config, TargetGeometry());
  GT_CHECK(!strict_detector.detect(image, 0).valid);

  DetectionConfig permissive_config;
  permissive_config.score_threshold = 0.05f;
  TargetDetector permissive_detector(permissive_config, TargetGeometry());
  const Detection detection = permissive_detector.detect(image, 0);
  GT_CHECK(detection.valid);
  GT_CHECK(detection.score >= 0.05f);
}

// ---------------------------------------------------------------------------
// Debug outputs / lifecycle.
// ---------------------------------------------------------------------------
GT_TEST(detector_debug_outputs) {
  TargetDetector detector = makeDetector();
  const cv::Size size(640, 480);
  TargetSpec spec;
  spec.center = cv::Point2f(300.f, 250.f);
  spec.radius = 25.f;
  const cv::Mat image = renderTargetScene(size, spec);

  GT_CHECK(detector.debugMask().empty());  // empty before detect
  const Detection detection = detector.detect(image, 5);
  GT_CHECK(detection.valid);
  GT_CHECK(!detector.debugMask().empty());
  GT_CHECK(!detector.debugCandidates().empty());
  GT_CHECK(detector.debugMask().cols == size.width);
  GT_CHECK(detector.debugMask().rows == size.height);
  GT_CHECK(detector.debugCandidates().cols == size.width);
  GT_CHECK(detector.debugCandidates().rows == size.height);
  GT_CHECK(detector.debugCandidates().type() == CV_8UC3);

  // Candidate list is populated and contains the winning (accepted) candidate.
  const auto& candidates = detector.debugCandidateInfo();
  GT_CHECK(!candidates.empty());
  bool has_accepted = false;
  for (const auto& candidate : candidates) {
    GT_CHECK(candidate.score >= 0.f && candidate.score <= 1.f);
    if (candidate.accepted) has_accepted = true;
  }
  GT_CHECK(has_accepted);

  // Debug rendering can be disabled (production / timing runs).
  detector.setDebugEnabled(false);
  GT_CHECK(!detector.debugEnabled());
  const Detection detection2 = detector.detect(image, 6);
  GT_CHECK(detection2.valid);
  GT_CHECK(detector.debugMask().empty());
  GT_CHECK(detector.debugCandidates().empty());
  GT_CHECK(!detector.debugCandidateInfo().empty());
  detector.setDebugEnabled(true);
}

GT_TEST(detector_reset_clears_state) {
  TargetDetector detector = makeDetector();
  TargetSpec spec;
  spec.center = cv::Point2f(320.f, 240.f);
  spec.radius = 26.f;
  const cv::Mat image = renderTargetScene(cv::Size(640, 480), spec);
  GT_CHECK(detector.detect(image, 0).valid);
  detector.reset();
  GT_CHECK(detector.debugMask().empty());
  GT_CHECK(detector.debugCandidates().empty());
  GT_CHECK(detector.debugCandidateInfo().empty());
  // Detector keeps working after reset.
  GT_CHECK(detector.detect(image, 1).valid);
}

// BGRA inputs are normalised to BGR internally; a grayscale frame carries no
// colour information, so it can never produce a valid detection but must be
// handled gracefully (no crash, invalid result).
GT_TEST(detector_accepts_bgra_and_handles_gray) {
  TargetDetector detector = makeDetector();
  TargetSpec spec;
  spec.center = cv::Point2f(320.f, 240.f);
  spec.radius = 28.f;
  const cv::Mat bgr = renderTargetScene(cv::Size(640, 480), spec);
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Mat bgra;
  cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
  const Detection bgra_detection = detector.detect(bgra, 1);
  GT_CHECK(bgra_detection.valid);
  GT_CHECK_NEAR(bgra_detection.disc_ellipse.center.x, spec.center.x, 3.0f);
  GT_CHECK_NEAR(bgra_detection.disc_ellipse.semi_major, spec.radius,
                0.12f * spec.radius);
  const Detection gray_detection = detector.detect(gray, 0);
  GT_CHECK(!gray_detection.valid);
  GT_CHECK(gray_detection.frame_index == 0);
}

// ---------------------------------------------------------------------------
// Glow geometry sanity: the reported glow ellipse / bbox surround the disc.
// ---------------------------------------------------------------------------
GT_TEST(detector_glow_geometry_is_plausible) {
  TargetDetector detector = makeDetector();
  TargetSpec spec;
  spec.center = cv::Point2f(320.f, 240.f);
  spec.radius = 30.f;
  const cv::Mat image = renderTargetScene(cv::Size(640, 480), spec);
  const Detection detection = detector.detect(image, 0);
  GT_CHECK(detection.valid);
  GT_CHECK(detection.glow_ellipse.valid());
  GT_CHECK_GT(detection.glow_ellipse.semi_major,
              detection.disc_ellipse.semi_major);
  GT_CHECK_GT(detection.bbox.width, 0.f);
  GT_CHECK_GT(detection.bbox.height, 0.f);
  GT_CHECK(detection.bbox.width >=
           1.5f * detection.disc_ellipse.semi_major);
  GT_CHECK_GT(detection.area_px, 0.f);
}
