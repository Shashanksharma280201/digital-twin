#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <vector>

#include "dt_lidar_bridge/canonical.hpp"

using dt_lidar_bridge::RawPoint;
using sensor_msgs::msg::PointField;

namespace
{
float read_f32(const sensor_msgs::msg::PointCloud2 & msg, size_t i, uint32_t offset)
{
  float v;
  std::memcpy(&v, msg.data.data() + i * msg.point_step + offset, 4);
  return v;
}

// Airy-style XYZIRT cloud: float32 xyz/intensity, uint16 ring, float64 absolute seconds.
sensor_msgs::msg::PointCloud2 make_xyzirt(const std::vector<std::array<double, 5>> & pts)
{
  sensor_msgs::msg::PointCloud2 msg;
  auto add = [&msg](const char * name, uint32_t off, uint8_t type) {
      PointField f;
      f.name = name;
      f.offset = off;
      f.datatype = type;
      f.count = 1;
      msg.fields.push_back(f);
    };
  add("x", 0, PointField::FLOAT32);
  add("y", 4, PointField::FLOAT32);
  add("z", 8, PointField::FLOAT32);
  add("intensity", 12, PointField::FLOAT32);
  add("ring", 16, PointField::UINT16);
  add("timestamp", 18, PointField::FLOAT64);
  msg.point_step = 26;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(pts.size());
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);
  for (size_t i = 0; i < pts.size(); ++i) {
    uint8_t * p = msg.data.data() + i * msg.point_step;
    const float xyz[4] = {static_cast<float>(pts[i][0]), static_cast<float>(pts[i][1]),
      static_cast<float>(pts[i][2]), 7.0f};
    const uint16_t ring = static_cast<uint16_t>(pts[i][3]);
    const double t = pts[i][4];
    std::memcpy(p, xyz, 16);
    std::memcpy(p + 16, &ring, 2);
    std::memcpy(p + 18, &t, 8);
  }
  return msg;
}
}  // namespace

TEST(Canonical, AbsoluteTimesBecomeSortedRelative)
{
  const double t_abs = 1760000000.25;
  auto in = make_xyzirt(
    {{{1, 0, 0, 3, t_abs + 0.05}}, {{2, 0, 0, 1, t_abs}}, {{3, 0, 0, 2, t_abs + 0.09}}});
  std::vector<RawPoint> pts;
  int64_t base_ns = 0;
  ASSERT_TRUE(dt_lidar_bridge::decode_pointcloud2(in, {}, pts, base_ns, nullptr));
  ASSERT_EQ(pts.size(), 3u);

  sensor_msgs::msg::PointCloud2 out;
  ASSERT_EQ(dt_lidar_bridge::build_canonical(pts, base_ns, {}, "lidar_link", out), 3u);
  EXPECT_EQ(out.header.frame_id, "lidar_link");
  EXPECT_NEAR(dt_lidar_bridge::stamp_to_ns(out.header.stamp) * 1e-9, t_abs, 1e-6);
  EXPECT_FLOAT_EQ(read_f32(out, 0, 0), 2.0f);  // earliest point first
  EXPECT_NEAR(read_f32(out, 0, 16), 0.0f, 1e-6);
  EXPECT_NEAR(read_f32(out, 1, 16), 0.05f, 1e-5);
  EXPECT_NEAR(read_f32(out, 2, 16), 0.09f, 1e-5);
  uint16_t ring;
  std::memcpy(&ring, out.data.data() + 20, 2);
  EXPECT_EQ(ring, 1);
}

TEST(Canonical, RangeAndNanFiltering)
{
  std::vector<RawPoint> pts{
    {0.05f, 0, 0, 0, 0.0, 0},            // inside blind range
    {NAN, 0, 0, 0, 0.01, 0},             // invalid
    {5.0f, 0, 0, 0, 0.02, 0},
    {500.0f, 0, 0, 0, 0.03, 0}};         // beyond max range
  sensor_msgs::msg::PointCloud2 out;
  dt_lidar_bridge::CanonicalOptions opt;
  EXPECT_EQ(dt_lidar_bridge::build_canonical(pts, 1000000000LL, opt, "lidar_link", out), 1u);
  EXPECT_EQ(dt_lidar_bridge::stamp_to_ns(out.header.stamp), 1000000000LL + 20000000LL);
}

TEST(Canonical, LargeShuffledScanIsExactlySorted)
{
  std::vector<RawPoint> pts;
  uint32_t seed = 12345;
  for (int i = 0; i < 50000; ++i) {
    seed = seed * 1664525u + 1013904223u;
    const double t = (seed % 1000000) * 1e-7;  // duplicates and clusters included
    pts.push_back({1.0f + i % 7, 0.0f, 0.0f, static_cast<float>(i), t, 0});
  }
  sensor_msgs::msg::PointCloud2 out;
  ASSERT_EQ(dt_lidar_bridge::build_canonical(pts, 0, {}, "lidar_link", out), 50000u);
  float prev = -1.0f;
  for (size_t i = 0; i < out.width; ++i) {
    const float t = read_f32(out, i, 16);
    ASSERT_GE(t, prev) << "at " << i;
    prev = t;
  }
}

TEST(Canonical, MissingTimeFieldFails)
{
  auto in = make_xyzirt({{{1, 0, 0, 0, 1.0}}});
  dt_lidar_bridge::Pc2DecodeOptions opt;
  opt.time_field = "t";
  std::vector<RawPoint> pts;
  int64_t base_ns = 0;
  std::string error;
  EXPECT_FALSE(dt_lidar_bridge::decode_pointcloud2(in, opt, pts, base_ns, &error));
  EXPECT_NE(error.find("'t'"), std::string::npos);
}

TEST(Canonical, ImuScaleAndRotation)
{
  // Airy IMU mounting rotation (rslidar_sdk #203).
  const std::array<double, 9> R{0, -1, 0, -1, 0, 0, 0, 0, -1};
  sensor_msgs::msg::Imu in;
  in.linear_acceleration.z = -1.0;  // upside-down IMU at rest reads -1 g
  in.angular_velocity.x = 0.5;
  sensor_msgs::msg::Imu out;
  dt_lidar_bridge::transform_imu(in, R, 9.80665, 1.0, "body", out);
  EXPECT_NEAR(out.linear_acceleration.z, 9.80665, 1e-9);
  EXPECT_NEAR(out.angular_velocity.y, -0.5, 1e-12);
  EXPECT_EQ(out.header.frame_id, "body");
  EXPECT_EQ(out.orientation_covariance[0], -1.0);
}
