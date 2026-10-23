// Synthetic validation dataset generator for the green glow tracker.
//
// Renders the green glowing target (white 3D-printed V-bracket + green glowing
// diffuser + bright white centre disc used for PnP) at KNOWN 3D poses into PNG
// image sequences, together with a per-frame ground-truth CSV.
//
//   glowtrack_synth --scenario <static_sweep|dynamic|stress> --out <dir>
//                   [--frames N] [--width 640 --height 480 --fx 500 --fy 500
//                    --cx -1 --cy -1] [--seed 42] [--noise sigma]
//                   [--blur px] [--fps 30] [--disc-mm 50] [--glow-mm 62]
//                   [--no-clutter] [--quiet]
//
// Output layout:
//   <out>/frames/frame_00000.png ...
//   <out>/gt.csv
//   <out>/meta.yaml
//
// Ground-truth conventions follow include/glowtrack/types.hpp:
//   yaw   = atan2(t.x, t.z)   [deg], positive = target to the right of the axis
//   pitch = -atan2(t.y, t.z)  [deg], positive = target above the axis
//   distance = ||t||          [m]
// The target plane is oriented so that its normal points back at the camera
// (i.e. the "camera facing" case), then rolled in-plane (roll_deg) and tilted
// about its horizontal axis (tilt_deg) to make the observed ellipse oblique.
//
// The ground-truth disc_cx/disc_cy/disc_radius_px columns are the ellipse that
// a perfect detector would fit to the rendered white disc (fitEllipse of the
// projected circle), so they can be compared directly against the CSV columns
// ellipse_cx/ellipse_cy/semi_major/semi_minor of the tracker.
//
// This tool is owned by the validation workstream; see docs/validation_synthetic.md.

#include "glowtrack/util.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

namespace {

using glowtrack::kDeg2Rad;
using glowtrack::kRad2Deg;

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Command line
// ---------------------------------------------------------------------------
struct Options {
  std::string scenario;
  std::string out;
  int frames = -1;  // -1 = scenario default
  int width = 640;
  int height = 480;
  double fx = 500.0;
  double fy = -1.0;  // -1 = fx
  double cx = -1.0;  // -1 = width/2
  double cy = -1.0;  // -1 = height/2
  int seed = 42;
  double noise = -1.0;  // -1 = scenario default
  double blur = -1.0;   // -1 = scenario default (fixed length in px)
  double fps = 30.0;
  double disc_mm = 50.0;
  double glow_mm = 62.0;
  bool clutter = true;
  bool quiet = false;
};

void printUsage() {
  std::printf(
      "glowtrack_synth -- synthetic validation dataset generator\n\n"
      "Usage:\n"
      "  glowtrack_synth --scenario <static_sweep|dynamic|stress> --out <dir>\n"
      "                  [--frames N] [--width 640] [--height 480]\n"
      "                  [--fx 500] [--fy 500] [--cx 320] [--cy 240]\n"
      "                  [--seed 42] [--noise sigma] [--blur px] [--fps 30]\n"
      "                  [--disc-mm 50] [--glow-mm 62] [--no-clutter] [--quiet]\n\n"
      "Scenarios:\n"
      "  static_sweep  deterministic pose grid: yaw -25..25 deg, pitch -20..20,\n"
      "                range 0.4..1.6 m, roll/tilt 0..40 deg, 3 frames per pose\n"
      "  dynamic       smooth sinusoidal yaw/pitch/range/roll motion with three\n"
      "                detection dropouts (target not rendered) for filter tests\n"
      "  stress        small far target (disc ~10-20 px), heavy blur/noise, and\n"
      "                partial occlusion by a dark rectangle\n\n"
      "Outputs: <out>/frames/frame_XXXXX.png, <out>/gt.csv, <out>/meta.yaml\n");
}

bool parseInt(const char* text, int* value) {
  char* end = nullptr;
  const long v = std::strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  *value = static_cast<int>(v);
  return true;
}

bool parseDouble(const char* text, double* value) {
  char* end = nullptr;
  const double v = std::strtod(text, &end);
  if (end == text || *end != '\0') {
    return false;
  }
  *value = v;
  return true;
}

bool parseArgs(int argc, char** argv, Options* opt) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: missing value for %s\n", name);
        return nullptr;
      }
      return argv[++i];
    };
    if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else if (arg == "--scenario") {
      const char* v = next("--scenario");
      if (!v) return false;
      opt->scenario = v;
    } else if (arg == "--out") {
      const char* v = next("--out");
      if (!v) return false;
      opt->out = v;
    } else if (arg == "--frames") {
      const char* v = next("--frames");
      if (!v || !parseInt(v, &opt->frames)) {
        std::fprintf(stderr, "error: bad --frames value\n");
        return false;
      }
    } else if (arg == "--width") {
      const char* v = next("--width");
      if (!v || !parseInt(v, &opt->width)) return false;
    } else if (arg == "--height") {
      const char* v = next("--height");
      if (!v || !parseInt(v, &opt->height)) return false;
    } else if (arg == "--fx") {
      const char* v = next("--fx");
      if (!v || !parseDouble(v, &opt->fx)) return false;
    } else if (arg == "--fy") {
      const char* v = next("--fy");
      if (!v || !parseDouble(v, &opt->fy)) return false;
    } else if (arg == "--cx") {
      const char* v = next("--cx");
      if (!v || !parseDouble(v, &opt->cx)) return false;
    } else if (arg == "--cy") {
      const char* v = next("--cy");
      if (!v || !parseDouble(v, &opt->cy)) return false;
    } else if (arg == "--seed") {
      const char* v = next("--seed");
      if (!v || !parseInt(v, &opt->seed)) return false;
    } else if (arg == "--noise") {
      const char* v = next("--noise");
      if (!v || !parseDouble(v, &opt->noise)) return false;
    } else if (arg == "--blur") {
      const char* v = next("--blur");
      if (!v || !parseDouble(v, &opt->blur)) return false;
    } else if (arg == "--fps") {
      const char* v = next("--fps");
      if (!v || !parseDouble(v, &opt->fps)) return false;
    } else if (arg == "--disc-mm") {
      const char* v = next("--disc-mm");
      if (!v || !parseDouble(v, &opt->disc_mm)) return false;
    } else if (arg == "--glow-mm") {
      const char* v = next("--glow-mm");
      if (!v || !parseDouble(v, &opt->glow_mm)) return false;
    } else if (arg == "--no-clutter") {
      opt->clutter = false;
    } else if (arg == "--quiet") {
      opt->quiet = true;
    } else {
      std::fprintf(stderr, "error: unknown argument '%s' (try --help)\n",
                   arg.c_str());
      return false;
    }
  }
  if (opt->scenario.empty() || opt->out.empty()) {
    std::fprintf(stderr, "error: --scenario and --out are required\n");
    return false;
  }
  if (opt->scenario != "static_sweep" && opt->scenario != "dynamic" &&
      opt->scenario != "stress") {
    std::fprintf(stderr,
                 "error: unknown scenario '%s' (static_sweep|dynamic|stress)\n",
                 opt->scenario.c_str());
    return false;
  }
  if (opt->width < 64 || opt->height < 64 || opt->fx <= 1.0 || opt->fps <= 0.0) {
    std::fprintf(stderr, "error: invalid image/camera parameters\n");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Camera / pose model
// ---------------------------------------------------------------------------
struct Camera {
  int width = 640;
  int height = 480;
  double fx = 500.0;
  double fy = 500.0;
  double cx = 320.0;
  double cy = 240.0;

  cv::Point2d project(const cv::Vec3d& p_cam) const {
    return cv::Point2d(fx * p_cam[0] / p_cam[2] + cx,
                       fy * p_cam[1] / p_cam[2] + cy);
  }
};

// Kinematic pose description (the ground-truth values written to gt.csv).
struct Pose {
  double yaw_deg = 0.0;
  double pitch_deg = 0.0;
  double distance_m = 1.0;
  double roll_deg = 0.0;  // in-plane rotation about the target normal
  double tilt_deg = 0.0;  // out-of-plane tilt about the target x axis
};

// Full 3D target pose: target -> camera (OpenCV solvePnP convention).
struct Pose3D {
  cv::Vec3d t_mm{0, 0, 0};    // disc centre in the camera frame [mm]
  cv::Matx33d R;              // target -> camera rotation
  cv::Vec3d normal{0, 0, -1}; // target z axis in camera coordinates
};

// Places the disc centre exactly at the requested (yaw, pitch, distance):
//   atan2(t.x, t.z) = yaw,  -atan2(t.y, t.z) = pitch,  ||t|| = distance.
Pose3D makePose3D(const Pose& pose) {
  Pose3D out;
  const double yaw = pose.yaw_deg * kDeg2Rad;
  const double pitch = pose.pitch_deg * kDeg2Rad;
  cv::Vec3d dir(std::tan(yaw), -std::tan(pitch), 1.0);
  dir /= cv::norm(dir);
  out.t_mm = dir * (pose.distance_m * 1000.0);

  // Target normal points back at the camera (the bracket faces the shooter).
  const cv::Vec3d n = -dir;
  const cv::Vec3d cam_up(0.0, -1.0, 0.0);
  cv::Vec3d x_axis = cam_up.cross(n);
  const double xn = cv::norm(x_axis);
  x_axis = (xn > 1e-9) ? x_axis / xn : cv::Vec3d(1.0, 0.0, 0.0);
  const cv::Vec3d y_axis = n.cross(x_axis);  // camera "up" axis on the target

  // In-plane roll about the normal.
  const double roll = pose.roll_deg * kDeg2Rad;
  const cv::Vec3d xr = x_axis * std::cos(roll) + y_axis * std::sin(roll);
  const cv::Vec3d yr = y_axis * std::cos(roll) - x_axis * std::sin(roll);
  // Out-of-plane tilt about the (rolled) target x axis.
  const double tilt = pose.tilt_deg * kDeg2Rad;
  const cv::Vec3d yt = yr * std::cos(tilt) - n * std::sin(tilt);
  const cv::Vec3d nt = yr * std::sin(tilt) + n * std::cos(tilt);

  out.R = cv::Matx33d(xr[0], yt[0], nt[0],
                      xr[1], yt[1], nt[1],
                      xr[2], yt[2], nt[2]);
  out.normal = nt;
  return out;
}

cv::Vec3d targetPointToCamera(const Pose3D& pose, double x_mm, double y_mm,
                              double z_mm = 0.0) {
  return pose.R * cv::Vec3d(x_mm, y_mm, z_mm) + pose.t_mm;
}

// Projects a polygon that lies in the target plane (z = 0). Returns an empty
// vector if any vertex falls behind the camera (never happens in our setups).
std::vector<cv::Point2f> projectPlanarPolygon(const Pose3D& pose,
                                              const Camera& cam,
                                              const std::vector<cv::Point2d>& pts_mm) {
  std::vector<cv::Point2f> out;
  out.reserve(pts_mm.size());
  for (const auto& p : pts_mm) {
    const cv::Vec3d pc = targetPointToCamera(pose, p.x, p.y);
    if (pc[2] <= 10.0) {
      return {};
    }
    const cv::Point2d uv = cam.project(pc);
    out.emplace_back(static_cast<float>(uv.x), static_cast<float>(uv.y));
  }
  return out;
}

std::vector<cv::Point2f> projectCircle(const Pose3D& pose, const Camera& cam,
                                       double radius_mm, int samples = 96) {
  std::vector<cv::Point2d> pts_mm;
  pts_mm.reserve(static_cast<size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double a = 2.0 * kPi * i / samples;
    pts_mm.emplace_back(radius_mm * std::cos(a), radius_mm * std::sin(a));
  }
  return projectPlanarPolygon(pose, cam, pts_mm);
}

// The ellipse a perfect detector would fit to the rendered white disc.
bool projectedDiscEllipse(const Pose3D& pose, const Camera& cam,
                          double disc_radius_mm, cv::RotatedRect* out) {
  const auto pts = projectCircle(pose, cam, disc_radius_mm, 96);
  if (pts.size() < 8) {
    return false;
  }
  *out = cv::fitEllipse(pts);
  return true;
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------
void fillPolyAA(cv::Mat& img, const std::vector<cv::Point2f>& pts,
                const cv::Scalar& color) {
  if (pts.size() < 3) {
    return;
  }
  std::vector<std::vector<cv::Point2f>> polys{pts};
  cv::fillPoly(img, polys, color, cv::LINE_AA);
}

// Chamfered rectangle in the target plane (the white plastic bracket).
std::vector<cv::Point2d> bracketPolygon(double w, double h) {
  const double ch = std::min(w, h) * 0.12;
  const double x = w * 0.5;
  const double y = h * 0.5;
  return {
      {-x + ch, -y}, {x - ch, -y}, {x, -y + ch}, {x, y - ch},
      {x - ch, y},   {-x + ch, y}, {-x, y - ch}, {-x, -y + ch},
  };
}

// V-shaped green diffuser wings (target plane [mm], x right / y up).
std::vector<std::vector<cv::Point2d>> diffuserWings(double glow_radius_mm) {
  const double g = glow_radius_mm;
  std::vector<std::vector<cv::Point2d>> wings;
  wings.push_back({{-0.92 * g, 0.10 * g},
                   {-1.75 * g, 0.80 * g},
                   {-2.10 * g, -0.15 * g},
                   {-1.55 * g, -0.65 * g},
                   {-0.92 * g, -0.30 * g}});
  wings.push_back({{0.92 * g, 0.10 * g},
                   {1.75 * g, 0.80 * g},
                   {2.10 * g, -0.15 * g},
                   {1.55 * g, -0.65 * g},
                   {0.92 * g, -0.30 * g}});
  return wings;
}

void drawTarget(cv::Mat& frame, const Camera& cam, const Pose3D& pose,
                double disc_radius_mm, double glow_radius_mm) {
  const double disc_r = disc_radius_mm;
  const double glow_r = std::max(glow_radius_mm, disc_r * 1.05);

  // 1) Light-grey 3D-printed bracket plate behind the glow (kept below the
  //    detector's white-disc threshold on purpose: only the disc is saturated).
  const auto plate =
      projectPlanarPolygon(pose, cam, bracketPolygon(2.75 * disc_r, 2.25 * disc_r));
  fillPolyAA(frame, plate, cv::Scalar(186, 189, 192));
  // A slightly brighter rim makes the plate read as a separate 3D object.
  if (!plate.empty()) {
    std::vector<std::vector<cv::Point2f>> polys{plate};
    cv::polylines(frame, polys, true, cv::Scalar(160, 163, 166), 2, cv::LINE_AA);
  }

  // 2) Green glow layer: the diffuser wings + the glowing annulus disc. This
  //    layer is rendered separately, blurred heavily and added back to simulate
  //    the light bleed of the real LED diffuser.
  cv::Mat glow_layer = cv::Mat::zeros(frame.size(), CV_8UC3);
  for (const auto& wing_mm : diffuserWings(glow_r)) {
    const auto wing = projectPlanarPolygon(pose, cam, wing_mm);
    fillPolyAA(glow_layer, wing, cv::Scalar(35, 165, 55));
  }
  const auto glow_circle = projectCircle(pose, cam, glow_r, 96);
  fillPolyAA(glow_layer, glow_circle, cv::Scalar(30, 215, 55));

  const double glow_radius_px =
      (glow_circle.size() > 2)
          ? 0.5 * (cv::norm(glow_circle[0] - glow_circle[48]) * 0.5 +
                   cv::norm(glow_circle[24] - glow_circle[72]) * 0.5)
          : 10.0;
  const double sigma = std::max(1.4, 0.22 * glow_radius_px);
  cv::Mat blurred;
  const int ksize = std::max(3, static_cast<int>(std::lround(sigma * 4.0)) | 1);
  cv::GaussianBlur(glow_layer, blurred, cv::Size(ksize, ksize), sigma);
  cv::addWeighted(frame, 1.0, blurred, 0.9, 0.0, frame);

  // 3) Crisp green parts on top of the halo: wings, bright glow annulus, then
  //    the darker green ring right at the disc boundary.
  for (const auto& wing_mm : diffuserWings(glow_r)) {
    const auto wing = projectPlanarPolygon(pose, cam, wing_mm);
    fillPolyAA(frame, wing, cv::Scalar(30, 180, 45));
  }
  const auto glow_sharp = projectCircle(pose, cam, glow_r, 96);
  fillPolyAA(frame, glow_sharp, cv::Scalar(25, 225, 40));
  const auto ring = projectCircle(pose, cam, disc_r * 1.13, 96);
  fillPolyAA(frame, ring, cv::Scalar(15, 115, 25));

  // 4) Saturated white centre disc: the PnP circle (over-exposed LED disc).
  const auto disc = projectCircle(pose, cam, disc_r, 96);
  fillPolyAA(frame, disc, cv::Scalar(255, 255, 255));
}

void applyMotionBlur(cv::Mat& frame, double length_px, double angle_deg) {
  const int len = static_cast<int>(std::lround(length_px));
  if (len < 2) {
    return;
  }
  const int ksize = len | 1;  // odd kernel size
  cv::Mat kernel8 = cv::Mat::zeros(ksize, ksize, CV_8U);
  const double a = angle_deg * kDeg2Rad;
  const double half = 0.5 * (ksize - 1);
  const cv::Point2f c(static_cast<float>(half), static_cast<float>(half));
  const cv::Point2f d(static_cast<float>(half * std::cos(a)),
                      static_cast<float>(half * std::sin(a)));
  const cv::Point2f p0(c.x - d.x, c.y - d.y);
  const cv::Point2f p1(c.x + d.x, c.y + d.y);
  cv::line(kernel8, p0, p1, cv::Scalar(255), 1, cv::LINE_AA);
  cv::Mat kernel;
  kernel8.convertTo(kernel, CV_32F, 1.0 / 255.0);
  const double sum = cv::sum(kernel)[0];
  if (sum > 1e-6) {
    kernel /= sum;
  }
  cv::filter2D(frame, frame, -1, kernel);
}

// ---------------------------------------------------------------------------
// Background: neutral grey gradient + furniture-ish shapes + green clutter
// (false-positive bait) + one or two non-glowing white objects.
// ---------------------------------------------------------------------------
cv::Mat makeBackground(const Camera& cam, const Options& opt, unsigned seed) {
  cv::Mat bg(cam.height, cam.width, CV_8UC3);
  for (int y = 0; y < cam.height; ++y) {
    const double ty = static_cast<double>(y) / std::max(1, cam.height - 1);
    for (int x = 0; x < cam.width; ++x) {
      const double tx = static_cast<double>(x) / std::max(1, cam.width - 1);
      double v = 118.0 - 52.0 * ty + 7.0 * std::sin(2.0 * kPi * tx);
      v = std::clamp(v, 8.0, 245.0);
      bg.at<cv::Vec3b>(y, x) = cv::Vec3b(static_cast<uchar>(v - 2.0),
                                         static_cast<uchar>(v),
                                         static_cast<uchar>(v + 3.0));
    }
  }

  std::mt19937 rng(seed * 7919u + 13u);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  // Low-contrast furniture: a wall/cabinet band and a floor edge.
  cv::Mat overlay = bg.clone();
  cv::rectangle(overlay, cv::Rect(0, static_cast<int>(0.62 * cam.height),
                                  cam.width, static_cast<int>(0.20 * cam.height)),
                cv::Scalar(82, 86, 90), cv::FILLED);
  cv::addWeighted(overlay, 0.45, bg, 0.55, 0.0, bg);
  cv::line(bg, cv::Point(0, static_cast<int>(0.82 * cam.height)),
           cv::Point(cam.width, static_cast<int>(0.80 * cam.height)),
           cv::Scalar(55, 58, 62), 2, cv::LINE_AA);

  if (!opt.clutter) {
    return bg;
  }

  auto jitter = [&](double base, double amount) {
    return base + amount * (uni(rng) - 0.5);
  };

  // Green clutter blobs (detectable green pixels, but NOT glowing targets:
  // no saturated white disc, no clean green annulus).
  struct Blob {
    double cx, cy, ax, ay, angle;
    cv::Scalar color;
  };
  const std::vector<Blob> green_blobs = {
      {jitter(0.10, 0.03), jitter(0.22, 0.04), 0.075, 0.050, 20.0,
       cv::Scalar(30, 118, 42)},
      {jitter(0.88, 0.03), jitter(0.76, 0.04), 0.090, 0.055, -15.0,
       cv::Scalar(40, 130, 50)},
      {jitter(0.90, 0.02), jitter(0.20, 0.03), 0.035, 0.028, 5.0,
       cv::Scalar(25, 105, 35)},
      {jitter(0.30, 0.04), jitter(0.92, 0.02), 0.070, 0.030, 0.0,
       cv::Scalar(45, 140, 55)},
      {jitter(0.55, 0.04), jitter(0.12, 0.03), 0.045, 0.035, 30.0,
       cv::Scalar(35, 125, 38)},
  };
  for (const auto& b : green_blobs) {
    const cv::Point c(static_cast<int>(b.cx * cam.width),
                      static_cast<int>(b.cy * cam.height));
    const cv::Size axes(static_cast<int>(b.ax * cam.width),
                        static_cast<int>(b.ay * cam.height));
    cv::ellipse(bg, c, axes, b.angle, 0.0, 360.0, b.color, cv::FILLED,
                cv::LINE_AA);
  }
  // A couple of green rectangles (shelves / sticky notes) with rounded look.
  cv::rectangle(bg,
                cv::Rect(static_cast<int>(jitter(0.04, 0.02) * cam.width),
                         static_cast<int>(jitter(0.72, 0.04) * cam.height),
                         static_cast<int>(0.06 * cam.width),
                         static_cast<int>(0.05 * cam.height)),
                cv::Scalar(38, 122, 46), cv::FILLED, cv::LINE_AA);
  cv::rectangle(bg,
                cv::Rect(static_cast<int>(jitter(0.78, 0.03) * cam.width),
                         static_cast<int>(jitter(0.45, 0.05) * cam.height),
                         static_cast<int>(0.045 * cam.width),
                         static_cast<int>(0.10 * cam.height)),
                cv::Scalar(32, 112, 40), cv::FILLED, cv::LINE_AA);

  // White lamp-like objects: bright but with no green ring, so the detector
  // must reject them (tests the white-disc + green-annulus validation).
  cv::circle(bg,
             cv::Point(static_cast<int>(jitter(0.06, 0.02) * cam.width),
                       static_cast<int>(jitter(0.55, 0.05) * cam.height)),
             static_cast<int>(0.025 * cam.width), cv::Scalar(205, 208, 212),
             cv::FILLED, cv::LINE_AA);
  cv::rectangle(bg,
                cv::Rect(static_cast<int>(jitter(0.72, 0.03) * cam.width),
                         static_cast<int>(jitter(0.08, 0.03) * cam.height),
                         static_cast<int>(0.05 * cam.width),
                         static_cast<int>(0.04 * cam.height)),
                cv::Scalar(200, 204, 208), cv::FILLED, cv::LINE_AA);

  return bg;
}

cv::Mat makeVignette(const Camera& cam) {
  cv::Mat gray(cam.height, cam.width, CV_8UC1);
  const double cx = 0.5 * cam.width;
  const double cy = 0.5 * cam.height;
  const double max_r = std::hypot(cx, cy);
  for (int y = 0; y < cam.height; ++y) {
    for (int x = 0; x < cam.width; ++x) {
      const double r = std::hypot(x - cx, y - cy) / max_r;
      const double v = 255.0 * (1.0 - 0.22 * r * r);
      gray.at<uchar>(y, x) = static_cast<uchar>(std::clamp(v, 0.0, 255.0));
    }
  }
  std::vector<cv::Mat> channels{gray, gray, gray};
  cv::Mat color;
  cv::merge(channels, color);
  return color;
}

// ---------------------------------------------------------------------------
// Scenario definitions
// ---------------------------------------------------------------------------
int defaultFrames(const std::string& scenario) {
  if (scenario == "static_sweep") return 180;
  if (scenario == "dynamic") return 300;
  return 150;
}

double defaultNoise(const std::string& scenario) {
  if (scenario == "static_sweep") return 2.0;
  if (scenario == "dynamic") return 3.0;
  return 12.0;
}

// static_sweep: deterministic grid of 300 poses (yaw x pitch x range x
// roll/tilt), sampled with a stride so that a short sequence already covers the
// whole working envelope; 3 consecutive frames per pose.
Pose staticSweepPose(int frame) {
  static const double yaw_levels[5] = {-25.0, -15.0, 0.0, 15.0, 25.0};
  static const double pitch_levels[3] = {-20.0, 0.0, 20.0};
  static const double dist_levels[4] = {0.4, 0.8, 1.2, 1.6};
  static const double combos[5][2] = {
      {0.0, 0.0}, {20.0, 20.0}, {40.0, 40.0}, {0.0, 40.0}, {40.0, 0.0}};
  const int pose_index = frame / 3;
  const int idx = (pose_index * 137) % 300;  // 137 coprime with 300
  const int ci = idx % 5;
  int t = idx / 5;
  const int di = t % 4;
  t /= 4;
  const int pi = t % 3;
  const int yi = (t / 3) % 5;
  Pose p;
  p.yaw_deg = yaw_levels[yi];
  p.pitch_deg = pitch_levels[pi];
  p.distance_m = dist_levels[di];
  p.roll_deg = combos[ci][0];
  p.tilt_deg = combos[ci][1];
  return p;
}

Pose dynamicPose(int frame, double fps) {
  const double t = frame / fps;
  Pose p;
  p.yaw_deg = 20.0 * std::sin(2.0 * kPi * t / 6.0);
  p.pitch_deg = 12.0 * std::sin(2.0 * kPi * t / 5.0 + 0.7);
  p.distance_m = 0.85 + 0.35 * std::sin(2.0 * kPi * t / 8.0 + 1.2);
  p.roll_deg = 12.0 * std::sin(2.0 * kPi * t / 3.5);
  p.tilt_deg = 8.0 * std::sin(2.0 * kPi * t / 4.5 + 0.2);
  return p;
}

Pose stressPose(int frame, double fps) {
  const double t = frame / fps;
  Pose p;
  p.yaw_deg = 14.0 * std::sin(2.0 * kPi * t / 5.5);
  p.pitch_deg = 9.0 * std::sin(2.0 * kPi * t / 7.0 + 0.6);
  p.distance_m = 0.90 + 0.20 * std::sin(2.0 * kPi * t / 9.0 + 1.0);
  p.roll_deg = 25.0 * std::sin(2.0 * kPi * t / 4.2);
  p.tilt_deg = 15.0 * std::sin(2.0 * kPi * t / 6.0 + 0.3);
  return p;
}

std::vector<std::pair<int, int>> dynamicDropouts(int frames) {
  std::vector<std::pair<int, int>> dropouts;
  const double starts[3] = {0.20, 0.47, 0.72};
  const int lengths[3] = {10, 8, 6};
  for (int k = 0; k < 3; ++k) {
    const int s = static_cast<int>(std::llround(starts[k] * (frames - 1)));
    const int e = std::min(frames - 1, s + lengths[k] - 1);
    if (s < frames) {
      dropouts.emplace_back(s, e);
    }
  }
  return dropouts;
}

std::vector<std::pair<int, int>> stressOcclusions(int frames) {
  std::vector<std::pair<int, int>> intervals;
  const double starts[2] = {0.22, 0.66};
  const double ends[2] = {0.42, 0.84};
  for (int k = 0; k < 2; ++k) {
    const int s = static_cast<int>(std::llround(starts[k] * (frames - 1)));
    const int e = static_cast<int>(std::llround(ends[k] * (frames - 1)));
    if (s < frames) {
      intervals.emplace_back(s, std::min(e, frames - 1));
    }
  }
  return intervals;
}

bool inIntervals(const std::vector<std::pair<int, int>>& intervals, int frame) {
  for (const auto& iv : intervals) {
    if (frame >= iv.first && frame <= iv.second) {
      return true;
    }
  }
  return false;
}

std::string intervalsToString(const std::vector<std::pair<int, int>>& intervals) {
  std::string out;
  for (size_t i = 0; i < intervals.size(); ++i) {
    if (i > 0) out += ",";
    out += std::to_string(intervals[i].first) + "-" +
           std::to_string(intervals[i].second);
  }
  return out;
}

double exposureFactor(const std::string& scenario, int frame, double fps,
                      unsigned seed) {
  const double t = frame / fps;
  const double phase = static_cast<double>(seed % 100) * 0.01;
  if (scenario == "static_sweep") {
    return 1.0 + 0.03 * std::sin(2.0 * kPi * t * 0.7 + phase);
  }
  if (scenario == "dynamic") {
    return 1.0 + 0.05 * std::sin(2.0 * kPi * t / 3.1 + phase) +
           0.03 * std::sin(2.0 * kPi * t * 0.37 + 0.5 * phase);
  }
  return 0.92 + 0.08 * std::sin(2.0 * kPi * t / 2.7 + 0.4 + phase);
}

// ---------------------------------------------------------------------------
// meta.yaml
// ---------------------------------------------------------------------------
void writeMeta(const Options& opt, const Camera& cam, int frames,
               const std::vector<std::pair<int, int>>& dropouts,
               const std::vector<std::pair<int, int>>& occlusions) {
  const std::string path =
      (std::filesystem::path(opt.out) / "meta.yaml").string();
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  fs << "scenario" << opt.scenario;
  fs << "frames" << frames;
  fs << "fps" << opt.fps;
  fs << "width" << cam.width;
  fs << "height" << cam.height;
  fs << "fx" << cam.fx;
  fs << "fy" << cam.fy;
  fs << "cx" << cam.cx;
  fs << "cy" << cam.cy;
  fs << "seed" << opt.seed;
  fs << "noise_sigma" << opt.noise;
  fs << "motion_blur_px" << (opt.blur >= 0.0 ? opt.blur : -1.0)
     << "  # -1 = scenario default (dynamic: 0.5 * image displacement)";
  fs << "disc_diameter_mm" << opt.disc_mm;
  fs << "glow_outer_diameter_mm" << opt.glow_mm;
  fs << "clutter" << (opt.clutter ? 1 : 0);
  fs << "dropouts" << intervalsToString(dropouts);
  fs << "occlusions" << intervalsToString(occlusions);
  fs << "notes"
     << "Generated by glowtrack_synth; ground-truth angles follow "
        "include/glowtrack/types.hpp conventions.";
  fs.release();
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int run(const Options& opt_in) {
  Options opt = opt_in;
  if (opt.frames <= 0) {
    opt.frames = defaultFrames(opt.scenario);
  }
  if (opt.fy <= 0.0) {
    opt.fy = opt.fx;
  }
  if (opt.cx < 0.0) {
    opt.cx = 0.5 * opt.width;
  }
  if (opt.cy < 0.0) {
    opt.cy = 0.5 * opt.height;
  }
  if (opt.noise < 0.0) {
    opt.noise = defaultNoise(opt.scenario);
  }
  if (opt.frames < 1) {
    std::fprintf(stderr, "error: --frames must be positive\n");
    return 1;
  }

  Camera cam;
  cam.width = opt.width;
  cam.height = opt.height;
  cam.fx = opt.fx;
  cam.fy = opt.fy;
  cam.cx = opt.cx;
  cam.cy = opt.cy;

  const double disc_radius_mm = opt.disc_mm * 0.5;
  const double glow_radius_mm = opt.glow_mm * 0.5;

  // Scenario schedule.
  std::vector<std::pair<int, int>> dropouts;
  std::vector<std::pair<int, int>> occlusions;
  if (opt.scenario == "dynamic") {
    dropouts = dynamicDropouts(opt.frames);
  } else if (opt.scenario == "stress") {
    occlusions = stressOcclusions(opt.frames);
  }

  // Prepare output directories (and delete stale frames from a previous run).
  const std::filesystem::path out_dir(opt.out);
  const std::filesystem::path frames_dir = out_dir / "frames";
  std::error_code ec;
  std::filesystem::create_directories(frames_dir, ec);
  if (ec) {
    std::fprintf(stderr, "error: cannot create %s: %s\n",
                 frames_dir.string().c_str(), ec.message().c_str());
    return 1;
  }
  for (const auto& entry : std::filesystem::directory_iterator(frames_dir, ec)) {
    if (!entry.is_regular_file()) continue;
    const std::string name = entry.path().filename().string();
    if (name.rfind("frame_", 0) == 0 &&
        entry.path().extension() == ".png") {
      std::filesystem::remove(entry.path(), ec);
    }
  }

  std::mt19937 rng(static_cast<unsigned>(opt.seed) * 7919u + 13u);
  const cv::Mat background = makeBackground(cam, opt, static_cast<unsigned>(opt.seed));
  const cv::Mat vignette = makeVignette(cam);

  const std::string gt_path = (out_dir / "gt.csv").string();
  std::FILE* gt = std::fopen(gt_path.c_str(), "w");
  if (gt == nullptr) {
    std::fprintf(stderr, "error: cannot open %s\n", gt_path.c_str());
    return 1;
  }
  std::fprintf(gt,
               "frame,timestamp_s,yaw_deg,pitch_deg,distance_m,roll_deg,"
               "tilt_deg,visible,disc_cx,disc_cy,disc_radius_px\n");

  cv::Point2d prev_center(0.0, 0.0);
  bool have_prev = false;
  int visible_frames = 0;

  for (int frame = 0; frame < opt.frames; ++frame) {
    const double t = frame / opt.fps;

    Pose pose;
    if (opt.scenario == "static_sweep") {
      pose = staticSweepPose(frame);
    } else if (opt.scenario == "dynamic") {
      pose = dynamicPose(frame, opt.fps);
    } else {
      pose = stressPose(frame, opt.fps);
    }
    const bool visible = !inIntervals(dropouts, frame);
    const bool occluded = inIntervals(occlusions, frame);

    const Pose3D pose3d = makePose3D(pose);
    const cv::Point2d center(
        cam.fx * pose3d.t_mm[0] / pose3d.t_mm[2] + cam.cx,
        cam.fy * pose3d.t_mm[1] / pose3d.t_mm[2] + cam.cy);

    cv::RotatedRect disc_rr;
    const bool have_ellipse =
        projectedDiscEllipse(pose3d, cam, disc_radius_mm, &disc_rr);

    cv::Mat frame_img = background.clone();
    if (visible) {
      drawTarget(frame_img, cam, pose3d, disc_radius_mm, glow_radius_mm);
    }
    if (occluded && have_ellipse) {
      // Dark rectangular occluder over the right-hand ~60 % of the target.
      const double size = std::max(disc_rr.size.width, disc_rr.size.height);
      const double jx = 0.10 * size * (static_cast<double>(rng() % 1000) / 1000.0 - 0.5);
      const double jy = 0.10 * size * (static_cast<double>(rng() % 1000) / 1000.0 - 0.5);
      const int x0 = static_cast<int>(std::lround(disc_rr.center.x + 0.05 * size + jx));
      const int y0 = static_cast<int>(std::lround(disc_rr.center.y - 2.0 * size + jy));
      const int w = static_cast<int>(std::lround(2.6 * size));
      const int h = static_cast<int>(std::lround(4.0 * size));
      cv::rectangle(frame_img, cv::Rect(x0, y0, w, h),
                    cv::Scalar(28, 29, 31), cv::FILLED, cv::LINE_AA);
    }

    // Motion blur: direction from the inter-frame displacement of the target.
    double motion_len = 0.0;
    double motion_angle_deg = 10.0;
    if (have_prev) {
      const cv::Point2d d = center - prev_center;
      const double disp = std::hypot(d.x, d.y);
      motion_angle_deg = std::atan2(d.y, d.x) * kRad2Deg;
      if (opt.scenario == "dynamic") {
        motion_len = 0.5 * disp;
      } else if (opt.scenario == "stress") {
        motion_len = std::max(10.0, 0.4 * disp);
      }
    }
    if (opt.blur >= 0.0) {
      motion_len = opt.blur;
      if (!have_prev) motion_angle_deg = 10.0;
    }
    applyMotionBlur(frame_img, motion_len, motion_angle_deg);

    // Vignette, exposure variation, sensor noise.
    cv::multiply(frame_img, vignette, frame_img, 1.0 / 255.0);
    {
      cv::Mat tmp;
      frame_img.convertTo(tmp, CV_8U, exposureFactor(opt.scenario, frame,
                                                     opt.fps, opt.seed),
                          0.0);
      frame_img = tmp;
    }
    if (opt.noise > 0.0) {
      cv::theRNG().state = static_cast<uint64_t>(opt.seed) * 1000003ULL +
                           static_cast<uint64_t>(frame) * 7919ULL + 17ULL;
      cv::Mat noise(frame_img.size(), CV_8UC3);
      cv::randn(noise, cv::Scalar(0, 0, 0),
                cv::Scalar(opt.noise, opt.noise, opt.noise));
      cv::add(frame_img, noise, frame_img);
    }

    char name[64];
    std::snprintf(name, sizeof(name), "frame_%05d.png", frame);
    const std::string image_path = (frames_dir / name).string();
    if (!cv::imwrite(image_path, frame_img)) {
      std::fprintf(stderr, "error: cannot write %s\n", image_path.c_str());
      std::fclose(gt);
      return 1;
    }

    // Ground truth: ideal ellipse fit + true kinematic pose.
    const double cx = have_ellipse ? disc_rr.center.x : -1.0;
    const double cy = have_ellipse ? disc_rr.center.y : -1.0;
    const double radius_px =
        have_ellipse
            ? std::sqrt(0.25 * disc_rr.size.width * disc_rr.size.height)
            : -1.0;
    std::fprintf(gt,
                 "%d,%.6f,%.4f,%.4f,%.6f,%.4f,%.4f,%d,%.3f,%.3f,%.3f\n", frame,
                 t, pose.yaw_deg, pose.pitch_deg, pose.distance_m, pose.roll_deg,
                 pose.tilt_deg, visible ? 1 : 0, cx, cy, radius_px);
    if (visible) {
      ++visible_frames;
    }

    prev_center = center;
    have_prev = true;

    if (!opt.quiet && ((frame + 1) % 30 == 0 || frame + 1 == opt.frames)) {
      std::printf("\r[%s] %d/%d frames", opt.scenario.c_str(), frame + 1,
                  opt.frames);
      std::fflush(stdout);
    }
  }
  std::fclose(gt);
  if (!opt.quiet) {
    std::printf("\n");
  }

  writeMeta(opt, cam, opt.frames, dropouts, occlusions);

  std::printf("glowtrack_synth: scenario=%s frames=%d size=%dx%d fx=%.1f\n",
              opt.scenario.c_str(), opt.frames, cam.width, cam.height, cam.fx);
  std::printf("  visible frames : %d / %d\n", visible_frames, opt.frames);
  if (!dropouts.empty()) {
    std::printf("  dropouts       : %s\n", intervalsToString(dropouts).c_str());
  }
  if (!occlusions.empty()) {
    std::printf("  occlusions     : %s\n", intervalsToString(occlusions).c_str());
  }
  std::printf("  images         : %s\n",
              frames_dir.string().c_str());
  std::printf("  ground truth   : %s\n", gt_path.c_str());
  std::printf("  metadata       : %s\n",
              (out_dir / "meta.yaml").string().c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, &opt)) {
    return 1;
  }
  try {
    return run(opt);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
