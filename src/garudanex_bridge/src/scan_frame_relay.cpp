// Copyright 2026 Prince Patel. BSD-3-Clause.
//
// Rewrite the frame_id on incoming LaserScan messages.
//
// ros_gz_bridge copies Gazebo's own link name into header.frame_id verbatim.
// For the PX4 x500_lidar_2d model that name is "link", which does not exist in
// the GarudaNEX TF tree. SLAM Toolbox then cannot transform the scan and fails
// SILENTLY - no map, no error, nothing useful in the log. This node makes the
// mismatch explicit and fixable in one parameter instead of hiding it.
//
// On real hardware this node is simply not launched: the rplidar_ros driver
// takes frame_id as a parameter and stamps it correctly at source.

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace garudanex
{

class ScanFrameRelay : public rclcpp::Node
{
public:
  explicit ScanFrameRelay(const rclcpp::NodeOptions & options)
  : Node("garudanex_scan_frame_relay", options)
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "lidar_link");
    const auto in_topic  = declare_parameter<std::string>("input_topic", "/scan_raw");
    const auto out_topic = declare_parameter<std::string>("output_topic", "/scan");

    // SensorDataQoS is BEST_EFFORT + KEEP_LAST(5): the right profile for a
    // high-rate sensor where a dropped scan matters less than added latency.
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      out_topic, rclcpp::SensorDataQoS());

    // Taking a UniquePtr lets rclcpp move the message straight through when
    // intra-process comms are enabled, with no copy and no re-serialisation.
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      in_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
        if (!seen_) {
          seen_ = true;
          RCLCPP_INFO(get_logger(), "first scan received");
          RCLCPP_INFO(get_logger(), "  incoming frame_id : '%s'",
                      msg->header.frame_id.c_str());
          RCLCPP_INFO(get_logger(), "  rewriting to      : '%s'",
                      target_frame_.c_str());
          RCLCPP_INFO(get_logger(), "  fov  : %.3f .. %.3f rad (%.1f deg)",
                      msg->angle_min, msg->angle_max,
                      (msg->angle_max - msg->angle_min) * 180.0 / M_PI);
          RCLCPP_INFO(get_logger(), "  range: %.2f .. %.2f m",
                      msg->range_min, msg->range_max);
          RCLCPP_INFO(get_logger(), "  beams: %zu", msg->ranges.size());
        }
        msg->header.frame_id = target_frame_;
        ++count_;
        pub_->publish(std::move(msg));
      });

    diag_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      [this]() {
        const double rate = static_cast<double>(count_ - last_count_) / 5.0;
        if (!seen_) {
          RCLCPP_WARN(get_logger(),
                      "no scans yet - check the gz_topic_name in gz_bridge.yaml "
                      "against `gz topic -l`");
        } else {
          RCLCPP_INFO(get_logger(), "relaying %.1f Hz (total %lu) -> frame '%s'",
                      rate, count_, target_frame_.c_str());
        }
        last_count_ = count_;
      });

    RCLCPP_INFO(get_logger(), "scan frame relay up: %s -> %s (frame '%s')",
                in_topic.c_str(), out_topic.c_str(), target_frame_.c_str());
  }

private:
  std::string target_frame_;
  bool seen_{false};
  uint64_t count_{0}, last_count_{0};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace garudanex

RCLCPP_COMPONENTS_REGISTER_NODE(garudanex::ScanFrameRelay)
