#pragma once
// Core data types shared by every module of the green glow tracker.
//
// Conventions
// -----------
// Camera frame (OpenCV): x -> right, y -> down, z -> forward (optical axis).
// Target frame:          planar target, origin at the center of the white disc,
//                        z axis = plane normal pointing out of the target face.
// Measured angles:       yaw   = atan2(t.x, t.z)   [rad], positive = target right
//                        pitch = -atan2(t.y, t.z)  [rad], positive = target up
//                        distance = ||t||          [m]
// The target pose is always expressed as target -> camera (OpenCV solvePnP
// convention): X_cam = R * X_target + t.

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace glowtrack {

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Camera model
// ---------------------------------------------------------------------------
struct CameraIntrinsics {
  int width = 640;
  int height = 480;
  double fx = 500.0;
  double fy = 500.0;
  double cx = 320.0;
  double cy = 240.0;
  // OpenCV distortion coefficients (k1, k2, p1, p2, k3). Empty = ideal pinhole.
  std::vector<double> dist;

  cv::Mat cameraMatrix() const;  // CV_64F 3x3
  cv::Mat distortion() const;    // CV_64F 1xN, N = 0 or 5
};

// ---------------------------------------------------------------------------
// Target geometry (physical dimensions in millimetres)
// ---------------------------------------------------------------------------
struct TargetGeometry {
  // Diameter of the white circular disc in the center of the green glow.
  // This circle is the geometric primitive used by PnP.
  // NOTE: only the *distance* output scales with this value; yaw/pitch are
  // invariant to a uniform scale error of the target model.
  double disc_diameter_mm = 50.0;
  // Outer diameter of the green glowing diffuser (sanity checks / debug only).
  double glow_outer_diameter_mm = 62.0;
};

// ---------------------------------------------------------------------------
// 2-D ellipse in image coordinates (as returned by cv::fitEllipse)
// ---------------------------------------------------------------------------
struct Ellipse2D {
  cv::Point2f center{0.f, 0.f};
  float semi_major = 0.f;  // >= semi_minor
  float semi_minor = 0.f;
  float angle_deg = 0.f;  // orientation of the major axis, degrees

  float area() const;
  float mean_radius() const;  // sqrt(semi_major * semi_minor)
  bool valid() const;
};

// ---------------------------------------------------------------------------
// Per-frame detector output
// ---------------------------------------------------------------------------
struct Detection {
  bool valid = false;
  Ellipse2D disc_ellipse;  // white center disc boundary -> PnP circle
  Ellipse2D glow_ellipse;  // outer green glow (debug / scoring only)
  cv::Rect2f bbox;         // bounding box of the green glow in the image
  float area_px = 0.f;     // green glow area [px^2]
  float score = 0.f;       // candidate score, 0..1
  int frame_index = -1;
};

// ---------------------------------------------------------------------------
// PnP result
// ---------------------------------------------------------------------------
struct PoseEstimate {
  bool valid = false;
  cv::Vec3d rvec{0, 0, 0};  // target -> camera rotation (axis-angle)
  cv::Vec3d tvec{0, 0, 0};  // target center in camera frame [mm]
  cv::Matx33d R = cv::Matx33d::eye();
  double reproj_rmse_px = 0.0;
  double yaw = 0.0;         // [rad], positive target to the right
  double pitch = 0.0;       // [rad], positive target above optical axis
  double distance_m = 0.0;  // [m]
  int num_solutions = 0;    // number of solutions returned by IPPE
  bool ambiguous = false;   // facing test could not decide
  double solve_ms = 0.0;
};

// ---------------------------------------------------------------------------
// Tracker state machine
// ---------------------------------------------------------------------------
enum class TrackState {
  Idle = 0,
  Searching,
  Acquiring,
  Tracking,
  Predicting,
  Lost,
};

const char* toString(TrackState state);

// ---------------------------------------------------------------------------
// Final output consumed by the lower computer / logger
// ---------------------------------------------------------------------------
struct TrackOutput {
  bool valid = false;     // yaw/pitch usable by the lower computer
  bool measured = false;  // detection was used to update the filter this frame
  bool predicted = false; // angles extrapolated through a detection gap
  TrackState state = TrackState::Idle;
  int frame_index = -1;
  double timestamp_s = 0.0;
  double yaw = 0.0;        // [rad]
  double pitch = 0.0;      // [rad]
  double yaw_rate = 0.0;   // [rad/s]
  double pitch_rate = 0.0; // [rad/s]
  double distance_m = 0.0;
  double quality = 0.0;  // 0..1
  int track_id = -1;
};

// Packed serial packet sent to the lower computer (see docs/protocol.md).
#pragma pack(push, 1)
struct TrackPacket {
  uint8_t sof1 = 0xAA;
  uint8_t sof2 = 0x55;
  uint8_t version = 1;
  uint8_t seq = 0;
  // bit0 valid, bit1 measured, bit2 predicted, bit3 acquiring, bit4 tracking
  uint8_t flags = 0;
  int16_t yaw_q = 0;         // yaw   [1e-4 rad]
  int16_t pitch_q = 0;       // pitch [1e-4 rad]
  int16_t yaw_rate_q = 0;    // [1e-3 rad/s]
  int16_t pitch_rate_q = 0;  // [1e-3 rad/s]
  uint16_t distance_mm = 0;  // [mm], saturated at 65535
  uint8_t quality = 0;       // 0..100
  uint8_t reserved = 0;
  uint16_t crc16 = 0;  // CRC-16/CCITT-FALSE over bytes [0, size-2)
};
#pragma pack(pop)
static_assert(sizeof(TrackPacket) == 19, "TrackPacket must stay 19 bytes");

}  // namespace glowtrack
