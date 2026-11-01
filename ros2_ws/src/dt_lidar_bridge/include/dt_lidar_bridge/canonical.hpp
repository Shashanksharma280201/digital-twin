// Canonical digital-twin LiDAR contract (see docs/design §4.1).
//
// /dt/lidar/points : x, y, z, intensity (float32), time (float32, seconds since
// header.stamp, ascending), ring (uint16). header.stamp = earliest point.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace dt_lidar_bridge
{

// A point whose time is expressed in seconds relative to an int64 nanosecond base,
// which keeps sub-microsecond precision with absolute (epoch) clocks.
struct RawPoint
{
  float x;
  float y;
  float z;
  float intensity;
  double t;
  uint16_t ring;
};

struct CanonicalOptions
{
  double min_range = 0.1;
  double max_range = 200.0;
};

struct Pc2DecodeOptions
{
  std::string time_field = "timestamp";
  // Multiplier converting the stored time value to seconds (1e-9 for nanoseconds).
  double time_scale = 1.0;
  // true: per-point time is absolute; false: relative to header.stamp.
  bool time_absolute = true;
  std::string ring_field = "ring";
  std::string intensity_field = "intensity";
};

constexpr uint32_t kCanonicalPointStep = 24;

// Decodes an arbitrary PointCloud2 with per-point time. On success, `base_ns`
// is the time base that `RawPoint::t` is relative to.
bool decode_pointcloud2(
  const sensor_msgs::msg::PointCloud2 & in, const Pc2DecodeOptions & opt,
  std::vector<RawPoint> & out, int64_t & base_ns, std::string * error);

// Filters, time-sorts and packs points into the canonical layout. Returns the
// number of points written (0 means nothing usable; `out` is left untouched).
size_t build_canonical(
  std::vector<RawPoint> & pts, int64_t base_ns, const CanonicalOptions & opt,
  const std::string & frame_id, sensor_msgs::msg::PointCloud2 & out);

// Applies scale and rotation (row-major R, v_out = R * v_in) to an IMU sample.
void transform_imu(
  const sensor_msgs::msg::Imu & in, const std::array<double, 9> & R, double acc_scale,
  double gyro_scale, const std::string & frame_id, sensor_msgs::msg::Imu & out);

int64_t stamp_to_ns(const builtin_interfaces::msg::Time & t);
builtin_interfaces::msg::Time ns_to_stamp(int64_t ns);

}  // namespace dt_lidar_bridge
