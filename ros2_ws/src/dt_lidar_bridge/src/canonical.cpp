#include "dt_lidar_bridge/canonical.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numeric>

namespace dt_lidar_bridge
{
namespace
{
using sensor_msgs::msg::PointField;

const PointField * find_field(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

template<typename T>
double load(const uint8_t * p)
{
  T v;
  std::memcpy(&v, p, sizeof(T));
  return static_cast<double>(v);
}

double read_as_double(const uint8_t * p, uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8: return load<int8_t>(p);
    case PointField::UINT8: return load<uint8_t>(p);
    case PointField::INT16: return load<int16_t>(p);
    case PointField::UINT16: return load<uint16_t>(p);
    case PointField::INT32: return load<int32_t>(p);
    case PointField::UINT32: return load<uint32_t>(p);
    case PointField::FLOAT32: return load<float>(p);
    case PointField::FLOAT64: return load<double>(p);
    default: return std::nan("");
  }
}

void add_field(sensor_msgs::msg::PointCloud2 & msg, const std::string & name, uint32_t offset, uint8_t type)
{
  PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = type;
  f.count = 1;
  msg.fields.push_back(f);
}

// (time, index) pairs in ascending time. Scan times span a short window, so a counting
// sort on quantised time followed by an insertion pass is exact and O(n); a comparison
// sort costs ~20 ms per Airy scan.
std::vector<std::pair<double, uint32_t>> time_order(const std::vector<RawPoint> & pts)
{
  const size_t n = pts.size();
  const auto [lo, hi] = std::minmax_element(
    pts.begin(), pts.end(), [](const RawPoint & a, const RawPoint & b) {return a.t < b.t;});
  const double t_min = lo->t;
  const double span = hi->t - t_min;
  const double scale = span > 0.0 ? static_cast<double>(n - 1) / span : 0.0;
  auto bucket = [&](double t) {
      return std::min(static_cast<size_t>((t - t_min) * scale), n - 1);
    };

  std::vector<uint32_t> start(n + 1, 0);
  for (const auto & p : pts) {
    ++start[bucket(p.t) + 1];
  }
  std::partial_sum(start.begin(), start.end(), start.begin());
  std::vector<std::pair<double, uint32_t>> order(n);
  for (uint32_t i = 0; i < n; ++i) {
    order[start[bucket(pts[i].t)]++] = {pts[i].t, i};
  }
  for (size_t i = 1; i < n; ++i) {
    for (size_t j = i; j > 0 && order[j - 1].first > order[j].first; --j) {
      std::swap(order[j - 1], order[j]);
    }
  }
  return order;
}
}  // namespace

int64_t stamp_to_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

builtin_interfaces::msg::Time ns_to_stamp(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

bool decode_pointcloud2(
  const sensor_msgs::msg::PointCloud2 & in, const Pc2DecodeOptions & opt,
  std::vector<RawPoint> & out, int64_t & base_ns, std::string * error)
{
  auto fail = [error](const std::string & why) {
      if (error) {
        *error = why;
      }
      return false;
    };
  if (in.is_bigendian) {
    return fail("big-endian point clouds are not supported");
  }
  const PointField * fx = find_field(in, "x");
  const PointField * fy = find_field(in, "y");
  const PointField * fz = find_field(in, "z");
  const PointField * ft = find_field(in, opt.time_field);
  const PointField * fi = find_field(in, opt.intensity_field);
  const PointField * fr = find_field(in, opt.ring_field);
  if (!fx || !fy || !fz) {
    return fail("missing x/y/z fields");
  }
  if (!ft) {
    return fail("missing per-point time field '" + opt.time_field + "'");
  }

  const size_t n = static_cast<size_t>(in.width) * in.height;
  out.clear();
  out.reserve(n);

  bool have_base = false;
  double base_value = 0.0;
  if (!opt.time_absolute) {
    base_ns = stamp_to_ns(in.header.stamp);
  }

  for (uint32_t row = 0; row < in.height; ++row) {
    const uint8_t * row_ptr = in.data.data() + static_cast<size_t>(row) * in.row_step;
    for (uint32_t col = 0; col < in.width; ++col) {
      const uint8_t * p = row_ptr + static_cast<size_t>(col) * in.point_step;
      RawPoint pt;
      pt.x = static_cast<float>(read_as_double(p + fx->offset, fx->datatype));
      pt.y = static_cast<float>(read_as_double(p + fy->offset, fy->datatype));
      pt.z = static_cast<float>(read_as_double(p + fz->offset, fz->datatype));
      pt.intensity = fi ? static_cast<float>(read_as_double(p + fi->offset, fi->datatype)) : 0.0f;
      pt.ring = fr ? static_cast<uint16_t>(read_as_double(p + fr->offset, fr->datatype)) : 0;
      const double tv = read_as_double(p + ft->offset, ft->datatype);
      if (!std::isfinite(tv)) {
        continue;
      }
      if (opt.time_absolute) {
        if (!have_base) {
          base_value = tv;
          have_base = true;
        }
        pt.t = (tv - base_value) * opt.time_scale;
      } else {
        pt.t = tv * opt.time_scale;
      }
      out.push_back(pt);
    }
  }

  if (opt.time_absolute) {
    if (!have_base) {
      return fail("no finite per-point timestamps");
    }
    base_ns = std::llround(base_value * opt.time_scale * 1e9);
  }
  return true;
}

size_t build_canonical(
  std::vector<RawPoint> & pts, int64_t base_ns, const CanonicalOptions & opt,
  const std::string & frame_id, sensor_msgs::msg::PointCloud2 & out)
{
  const double min_r2 = opt.min_range * opt.min_range;
  const double max_r2 = opt.max_range * opt.max_range;
  auto rejected = [&](const RawPoint & p) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        return true;
      }
      const double r2 = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y +
        static_cast<double>(p.z) * p.z;
      return r2 < min_r2 || r2 > max_r2;
    };
  pts.erase(std::remove_if(pts.begin(), pts.end(), rejected), pts.end());
  if (pts.empty()) {
    return 0;
  }

  // FAST-LIO takes the last point's time as the scan end, so order matters.
  const std::vector<std::pair<double, uint32_t>> order = time_order(pts);
  const double t0 = order.front().first;

  out.header.stamp = ns_to_stamp(base_ns + std::llround(t0 * 1e9));
  out.header.frame_id = frame_id;
  out.height = 1;
  out.width = static_cast<uint32_t>(pts.size());
  out.is_bigendian = false;
  out.is_dense = true;
  out.point_step = kCanonicalPointStep;
  out.row_step = out.point_step * out.width;
  out.fields.clear();
  add_field(out, "x", 0, PointField::FLOAT32);
  add_field(out, "y", 4, PointField::FLOAT32);
  add_field(out, "z", 8, PointField::FLOAT32);
  add_field(out, "intensity", 12, PointField::FLOAT32);
  add_field(out, "time", 16, PointField::FLOAT32);
  add_field(out, "ring", 20, PointField::UINT16);
  out.data.assign(out.row_step, 0);

  uint8_t * dst = out.data.data();
  for (const auto & key : order) {
    const RawPoint & p = pts[key.second];
    const float time = static_cast<float>(p.t - t0);
    std::memcpy(dst + 0, &p.x, 4);
    std::memcpy(dst + 4, &p.y, 4);
    std::memcpy(dst + 8, &p.z, 4);
    std::memcpy(dst + 12, &p.intensity, 4);
    std::memcpy(dst + 16, &time, 4);
    std::memcpy(dst + 20, &p.ring, 2);
    dst += kCanonicalPointStep;
  }
  return pts.size();
}

void transform_imu(
  const sensor_msgs::msg::Imu & in, const std::array<double, 9> & R, double acc_scale,
  double gyro_scale, const std::string & frame_id, sensor_msgs::msg::Imu & out)
{
  auto rotate = [&R](double x, double y, double z, double s, geometry_msgs::msg::Vector3 & v) {
      v.x = s * (R[0] * x + R[1] * y + R[2] * z);
      v.y = s * (R[3] * x + R[4] * y + R[5] * z);
      v.z = s * (R[6] * x + R[7] * y + R[8] * z);
    };
  out.header.stamp = in.header.stamp;
  out.header.frame_id = frame_id;
  rotate(
    in.linear_acceleration.x, in.linear_acceleration.y, in.linear_acceleration.z, acc_scale,
    out.linear_acceleration);
  rotate(
    in.angular_velocity.x, in.angular_velocity.y, in.angular_velocity.z, gyro_scale,
    out.angular_velocity);
  out.orientation.x = 0.0;
  out.orientation.y = 0.0;
  out.orientation.z = 0.0;
  out.orientation.w = 1.0;
  out.orientation_covariance.fill(0.0);
  out.orientation_covariance[0] = -1.0;
  out.angular_velocity_covariance = in.angular_velocity_covariance;
  out.linear_acceleration_covariance = in.linear_acceleration_covariance;
}

}  // namespace dt_lidar_bridge
