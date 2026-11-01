// PointsBridge: driver point cloud -> canonical /dt/lidar/points.
//
// input_type "livox_custom": livox_ros_driver2/CustomMsg (Mid-360, xfer_format 1)
// input_type "pointcloud2" : any PointCloud2 with per-point time (RoboSense Airy XYZIRT)
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dt_lidar_bridge/canonical.hpp"

namespace dt_lidar_bridge
{

class PointsBridge : public rclcpp::Node
{
public:
  explicit PointsBridge(const rclcpp::NodeOptions & options)
  : Node("points_bridge", options)
  {
    input_type_ = declare_parameter<std::string>("input_type", "livox_custom");
    const auto input_topic = declare_parameter<std::string>("input_topic", "/livox/lidar");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/dt/lidar/points");
    frame_id_ = declare_parameter<std::string>("frame_id", "lidar_link");
    canon_.min_range = declare_parameter<double>("min_range", 0.1);
    canon_.max_range = declare_parameter<double>("max_range", 200.0);
    livox_tag_filter_ = declare_parameter<bool>("livox_tag_filter", true);
    decode_.time_field = declare_parameter<std::string>("time_field", "timestamp");
    decode_.time_scale = declare_parameter<double>("time_scale", 1.0);
    decode_.time_absolute = declare_parameter<bool>("time_absolute", true);
    decode_.ring_field = declare_parameter<std::string>("ring_field", "ring");
    decode_.intensity_field = declare_parameter<std::string>("intensity_field", "intensity");

    // Reliable publisher: compatible with best-effort (FAST-LIO, GLIM) and reliable (rosbag) readers.
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::QoS(10).reliable());

    if (input_type_ == "livox_custom") {
      livox_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic, rclcpp::SensorDataQoS(),
        [this](livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {on_livox(*msg);});
    } else if (input_type_ == "pointcloud2") {
      pc2_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {on_pointcloud2(*msg);});
    } else {
      throw std::invalid_argument("input_type must be 'livox_custom' or 'pointcloud2'");
    }
    RCLCPP_INFO(
      get_logger(), "%s [%s] -> %s (frame %s, range %.2f-%.1f m)", input_topic.c_str(),
      input_type_.c_str(), output_topic.c_str(), frame_id_.c_str(), canon_.min_range,
      canon_.max_range);
  }

private:
  void on_livox(const livox_ros_driver2::msg::CustomMsg & msg)
  {
    buffer_.clear();
    buffer_.reserve(msg.points.size());
    for (const auto & p : msg.points) {
      // Same return-confidence filter as FAST-LIO's Livox handler.
      if (livox_tag_filter_ && (p.tag & 0x30) != 0x10 && (p.tag & 0x30) != 0x00) {
        continue;
      }
      buffer_.push_back(
        RawPoint{p.x, p.y, p.z, static_cast<float>(p.reflectivity), p.offset_time * 1e-9,
          static_cast<uint16_t>(p.line)});
    }
    publish(static_cast<int64_t>(msg.timebase));
  }

  void on_pointcloud2(const sensor_msgs::msg::PointCloud2 & msg)
  {
    int64_t base_ns = 0;
    std::string error;
    if (!decode_pointcloud2(msg, decode_, buffer_, base_ns, &error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "dropping cloud: %s", error.c_str());
      return;
    }
    publish(base_ns);
  }

  void publish(int64_t base_ns)
  {
    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    if (build_canonical(buffer_, base_ns, canon_, frame_id_, *out) == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "empty scan after filtering");
      return;
    }
    pub_->publish(std::move(out));
  }

  std::string input_type_;
  std::string frame_id_;
  bool livox_tag_filter_{true};
  CanonicalOptions canon_;
  Pc2DecodeOptions decode_;
  std::vector<RawPoint> buffer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
};

}  // namespace dt_lidar_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(dt_lidar_bridge::PointsBridge)
