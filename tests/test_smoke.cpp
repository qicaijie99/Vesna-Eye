// Smoke tests for the shared plumbing (types / util / protocol). Keep this file
// dependency free; module tests live in test_detector.cpp, test_pose.cpp, ...
#include "test_framework.hpp"

#include "glowtrack/kalman_filter.hpp"
#include "glowtrack/types.hpp"
#include "glowtrack/util.hpp"

using namespace glowtrack;

GT_TEST(angle_wrapping) {
  GT_CHECK_NEAR(normalizeAngle(0.0), 0.0, 1e-12);
  GT_CHECK_NEAR(normalizeAngle(kPi), -kPi, 1e-9);
  GT_CHECK_NEAR(normalizeAngle(-kPi), -kPi, 1e-9);
  GT_CHECK_NEAR(normalizeAngle(3.0 * kPi), -kPi, 1e-9);
  GT_CHECK_NEAR(angleDifference(0.1, 6.2), 0.1 - 6.2 + 2.0 * kPi, 1e-9);
  GT_CHECK_NEAR(normalizeAnglePositive(-0.5), 2.0 * kPi - 0.5, 1e-9);
}

GT_TEST(camera_matrix_layout) {
  CameraIntrinsics camera;
  camera.fx = 500.0;
  camera.fy = 480.0;
  camera.cx = 320.0;
  camera.cy = 240.0;
  const cv::Mat k = camera.cameraMatrix();
  GT_CHECK(k.rows == 3 && k.cols == 3);
  GT_CHECK_NEAR(k.at<double>(0, 0), 500.0, 1e-12);
  GT_CHECK_NEAR(k.at<double>(1, 2), 240.0, 1e-12);
  const cv::Mat d = camera.distortion();
  GT_CHECK(d.cols == 5);
}

GT_TEST(ellipse_helpers) {
  Ellipse2D e;
  e.semi_major = 40.f;
  e.semi_minor = 30.f;
  GT_CHECK(e.valid());
  GT_CHECK_NEAR(e.mean_radius(), std::sqrt(40.0 * 30.0), 1e-3);
  GT_CHECK_NEAR(e.area(), kPi * 40.0 * 30.0, 1e-3);
}

GT_TEST(packet_size_and_crc) {
  GT_CHECK(sizeof(TrackPacket) == 19);
  TrackPacket packet;
  packet.yaw_q = 1234;
  const uint16_t crc = crc16Ccitt(reinterpret_cast<const uint8_t*>(&packet),
                                  sizeof(TrackPacket) - 2);
  const uint16_t crc_check = crc16Ccitt(reinterpret_cast<const uint8_t*>(&packet),
                                        sizeof(TrackPacket) - 2);
  GT_CHECK(crc == crc_check);
  // Classic CRC-16/CCITT-FALSE check value for "123456789" is 0x29B1.
  const char* check = "123456789";
  GT_CHECK_NEAR(crc16Ccitt(reinterpret_cast<const uint8_t*>(check), 9), 0x29B1,
                0.0);
}
