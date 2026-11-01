// ImuBridge: driver IMU (acceleration in g, sensor axes) -> canonical /dt/lidar/imu
// (m/s^2, axes parallel to lidar_link, frame "body").
#include <algorithm>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "dt_lidar_bridge/canonical.hpp"

namespace dt_lidar_bridge
{

class ImuBridge : public rclcpp::Node
{
public:
  explicit ImuBridge(const rclcpp::NodeOptions & options)
  : Node("imu_bridge", options)
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/livox/imu");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/dt/lidar/imu");
    frame_id_ = declare_parameter<std::string>("frame_id", "body");
    acc_scale_ = declare_parameter<double>("acc_scale", 9.80665);
    gyro_scale_ = declare_parameter<double>("gyro_scale", 1.0);
    const auto rot = declare_parameter<std::vector<double>>(
      "rotation", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    if (rot.size() != 9) {
      throw std::invalid_argument("rotation must have 9 elements (row-major)");
    }
    std::copy(rot.begin(), rot.end(), R_.begin());

    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, rclcpp::QoS(200).reliable());
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic, rclcpp::SensorDataQoS().keep_last(200),
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {
        auto out = std::make_unique<sensor_msgs::msg::Imu>();
        transform_imu(*msg, R_, acc_scale_, gyro_scale_, frame_id_, *out);
        pub_->publish(std::move(out));
      });
    RCLCPP_INFO(
      get_logger(), "%s -> %s (acc x%.5f, frame %s)", input_topic.c_str(), output_topic.c_str(),
      acc_scale_, frame_id_.c_str());
  }

private:
  std::string frame_id_;
  double acc_scale_{9.80665};
  double gyro_scale_{1.0};
  std::array<double, 9> R_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

}  // namespace dt_lidar_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(dt_lidar_bridge::ImuBridge)
