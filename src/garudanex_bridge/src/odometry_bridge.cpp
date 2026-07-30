// Copyright 2026 Prince Patel. BSD-3-Clause.
//
// GarudaNEX odometry bridge.
//
//   /fmu/out/vehicle_odometry  (PX4, NED world / FRD body)
//        |
//        +--> /odom                    nav_msgs/Odometry, ENU / FLU
//        +--> TF odom -> base_link     REP-105 continuous local estimate
//
// This node is ONE of exactly TWO places in GarudaNEX where a coordinate
// frame conversion happens (the other is the cmd_vel bridge). Keeping the
// conversion in one place is what makes sign errors findable.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "garudanex_bridge/frame_transforms.hpp"
#include "garudanex_bridge/px4_topics.hpp"

namespace garudanex
{

using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleLocalPosition;
namespace gf = garudanex::frames;

class OdometryBridge : public rclcpp::Node
{
public:
  explicit OdometryBridge(const rclcpp::NodeOptions & options)
  : Node("garudanex_odom_bridge", options)
  {
    odom_frame_      = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_      = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_      = declare_parameter<bool>("publish_tf", true);
    footprint_frame_ = declare_parameter<std::string>("base_footprint_frame", "base_footprint");
    publish_footprint_ = declare_parameter<bool>("publish_base_footprint", true);
    require_valid_   = declare_parameter<bool>("require_valid_estimate", true);
    pose_cov_floor_  = declare_parameter<double>("pose_covariance_floor", 1e-4);
    twist_cov_floor_ = declare_parameter<double>("twist_covariance_floor", 1e-4);

    // PX4 publishes BEST_EFFORT + TRANSIENT_LOCAL. The ROS 2 default
    // subscriber asks for RELIABLE + VOLATILE, which is INCOMPATIBLE - DDS
    // then simply never connects the endpoints. The topic appears in
    // `ros2 topic list`, `ros2 topic info` shows a publisher, and you receive
    // absolutely nothing, with no error anywhere. Matching this profile is
    // not optional.
    rmw_qos_profile_t profile = rmw_qos_profile_sensor_data;
    auto px4_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(profile.history, 5), profile);

    const auto odom_topic =
      px4::topicName<VehicleOdometry>("/fmu/out/vehicle_odometry");
    const auto lpos_topic =
      px4::topicName<VehicleLocalPosition>("/fmu/out/vehicle_local_position");

    odom_sub_ = create_subscription<VehicleOdometry>(
      odom_topic, px4_qos,
      std::bind(&OdometryBridge::onOdometry, this, std::placeholders::_1));

    lpos_sub_ = create_subscription<VehicleLocalPosition>(
      lpos_topic, px4_qos,
      std::bind(&OdometryBridge::onLocalPosition, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    diag_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&OdometryBridge::onDiagnostics, this));

    RCLCPP_INFO(get_logger(), "GarudaNEX odometry bridge up");
    RCLCPP_INFO(get_logger(), "  subscribing : %s", odom_topic.c_str());
    RCLCPP_INFO(get_logger(), "                %s", lpos_topic.c_str());
    RCLCPP_INFO(get_logger(), "  publishing  : odom  (%s -> %s)",
                odom_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  publish_tf=%s require_valid=%s",
                publish_tf_ ? "true" : "false",
                require_valid_ ? "true" : "false");
  }

private:
  // ---------------------------------------------------------------- validity
  void onLocalPosition(const VehicleLocalPosition::SharedPtr msg)
  {
    xy_valid_ = msg->xy_valid;
    z_valid_  = msg->z_valid;
  }

  // ---------------------------------------------------------------- main path
  void onOdometry(const VehicleOdometry::SharedPtr msg)
  {
    ++rx_count_;

    // Gate 1: does PX4 itself trust this estimate?
    if (require_valid_ && !(xy_valid_ && z_valid_)) {
      ++dropped_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "dropping odometry: estimate invalid (xy_valid=%d z_valid=%d)",
        static_cast<int>(xy_valid_), static_cast<int>(z_valid_));
      return;
    }

    // Gate 2: PX4 signals "unknown" with NaN. Never let NaN reach TF - it
    // poisons the whole tf2 buffer and every consumer downstream.
    if (!std::isfinite(msg->position[0]) || !std::isfinite(msg->position[1]) ||
        !std::isfinite(msg->position[2]) || !std::isfinite(msg->q[0]))
    {
      ++dropped_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                           "dropping odometry: non-finite pose");
      return;
    }

    // Gate 3: only NED-referenced odometry is handled. PX4 can also emit
    // FRD-referenced odometry; failing loudly beats converting it wrongly.
    if (msg->pose_frame != VehicleOdometry::POSE_FRAME_NED) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "unhandled pose_frame=%u (expected POSE_FRAME_NED=%u)",
                            msg->pose_frame,
                            static_cast<unsigned>(VehicleOdometry::POSE_FRAME_NED));
      return;
    }

    // Estimator reset: PX4 increments reset_counter whenever it discontinuously
    // jumps its position or heading. The pose after a reset is not continuous
    // with the pose before it, which violates the REP-105 promise that
    // odom->base_link never jumps. We log it loudly; SLAM's map->odom will
    // absorb the offset on its next optimisation.
    if (have_reset_ && msg->reset_counter != last_reset_) {
      ++resets_;
      RCLCPP_WARN(get_logger(),
                  "EKF2 estimator RESET (%u -> %u): odom frame just jumped",
                  last_reset_, msg->reset_counter);
    }
    last_reset_ = msg->reset_counter;
    have_reset_ = true;

    // ---- position: NED -> ENU ----
    const Eigen::Vector3d p_enu = gf::nedToEnu(
      {msg->position[0], msg->position[1], msg->position[2]});

    // ---- attitude: (NED->FRD) -> (ENU->FLU) ----
    const Eigen::Quaterniond q_enu_flu =
      gf::attitudeNedFrdToEnuFlu(gf::fromPx4Quat(msg->q));

    // ---- linear velocity ----
    // nav_msgs/Odometry requires twist expressed in the CHILD frame
    // (base_link), NOT the world frame. Nav2's controllers assume this.
    // Publishing world-frame velocity here yields a controller that only
    // behaves correctly while the drone happens to face East.
    Eigen::Vector3d v_body = Eigen::Vector3d::Zero();
    if (std::isfinite(msg->velocity[0])) {
      const Eigen::Vector3d v_raw{msg->velocity[0], msg->velocity[1],
                                  msg->velocity[2]};
      switch (msg->velocity_frame) {
        case VehicleOdometry::VELOCITY_FRAME_NED:
        case VehicleOdometry::VELOCITY_FRAME_FRD:
          // World-referenced: to ENU, then rotate into the body frame.
          v_body = q_enu_flu.inverse() * gf::nedToEnu(v_raw);
          break;
        case VehicleOdometry::VELOCITY_FRAME_BODY_FRD:
          // Already body-referenced: just flip FRD -> FLU.
          v_body = gf::frdToFlu(v_raw);
          break;
        default:
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "unknown velocity_frame=%u, zeroing twist",
                               msg->velocity_frame);
          break;
      }
    }

    // ---- angular velocity: body FRD -> body FLU ----
    Eigen::Vector3d w_flu = Eigen::Vector3d::Zero();
    if (std::isfinite(msg->angular_velocity[0])) {
      w_flu = gf::frdToFlu({msg->angular_velocity[0],
                            msg->angular_velocity[1],
                            msg->angular_velocity[2]});
    }

    // ---- assemble ----
    nav_msgs::msg::Odometry out;
    // Node clock, NOT msg->timestamp. PX4 timestamps are microseconds since
    // PX4 boot - not the ROS epoch and not simulation time. Using them
    // directly puts every transform decades away from now() and tf2 rejects
    // all of them.
    out.header.stamp    = this->now();
    out.header.frame_id = odom_frame_;
    out.child_frame_id  = base_frame_;

    out.pose.pose.position.x    = p_enu.x();
    out.pose.pose.position.y    = p_enu.y();
    out.pose.pose.position.z    = p_enu.z();
    out.pose.pose.orientation.w = q_enu_flu.w();
    out.pose.pose.orientation.x = q_enu_flu.x();
    out.pose.pose.orientation.y = q_enu_flu.y();
    out.pose.pose.orientation.z = q_enu_flu.z();

    out.twist.twist.linear.x  = v_body.x();
    out.twist.twist.linear.y  = v_body.y();
    out.twist.twist.linear.z  = v_body.z();
    out.twist.twist.angular.x = w_flu.x();
    out.twist.twist.angular.y = w_flu.y();
    out.twist.twist.angular.z = w_flu.z();

    fillPoseCovariance(msg, out.pose.covariance);
    fillTwistCovariance(msg, out.twist.covariance);

    odom_pub_->publish(out);
    ++tx_count_;

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp            = out.header.stamp;
      tf.header.frame_id         = odom_frame_;
      tf.child_frame_id          = base_frame_;
      tf.transform.translation.x = p_enu.x();
      tf.transform.translation.y = p_enu.y();
      tf.transform.translation.z = p_enu.z();
      tf.transform.rotation      = out.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
      if (publish_footprint_) {
        const auto & q = out.pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        geometry_msgs::msg::TransformStamped fp;
        fp.header.stamp            = out.header.stamp;
        fp.header.frame_id         = odom_frame_;
        fp.child_frame_id          = footprint_frame_;
        fp.transform.translation.x = p_enu.x();
        fp.transform.translation.y = p_enu.y();
        fp.transform.translation.z = 0.0;
        fp.transform.rotation.x    = 0.0;
        fp.transform.rotation.y    = 0.0;
        fp.transform.rotation.z    = std::sin(yaw * 0.5);
        fp.transform.rotation.w    = std::cos(yaw * 0.5);
        tf_broadcaster_->sendTransform(fp);
      }
    }
  }

  // ------------------------------------------------------------- covariance
  // PX4 reports per-axis VARIANCES for the NED axes, not a full matrix.
  // Remap the NED variance indices onto the ENU ordering of the ROS 6x6
  // (row-major, [x y z roll pitch yaw]): ENU x <- NED y, ENU y <- NED x.
  // A variance of exactly zero means "infinitely certain" and makes any
  // downstream filter that inverts the covariance produce NaN, so we floor it.
  void fillPoseCovariance(const VehicleOdometry::SharedPtr & msg,
                          std::array<double, 36> & cov) const
  {
    cov.fill(0.0);
    auto safe = [&](float v) {
      return std::max(std::isfinite(v) ? static_cast<double>(v)
                                       : pose_cov_floor_, pose_cov_floor_);
    };
    cov[0]  = safe(msg->position_variance[1]);      // ENU x  <- NED y
    cov[7]  = safe(msg->position_variance[0]);      // ENU y  <- NED x
    cov[14] = safe(msg->position_variance[2]);      // ENU z  <- NED z
    cov[21] = safe(msg->orientation_variance[1]);   // roll   <- pitch
    cov[28] = safe(msg->orientation_variance[0]);   // pitch  <- roll
    cov[35] = safe(msg->orientation_variance[2]);   // yaw
  }

  void fillTwistCovariance(const VehicleOdometry::SharedPtr & msg,
                           std::array<double, 36> & cov) const
  {
    cov.fill(0.0);
    auto safe = [&](float v) {
      return std::max(std::isfinite(v) ? static_cast<double>(v)
                                       : twist_cov_floor_, twist_cov_floor_);
    };
    cov[0]  = safe(msg->velocity_variance[1]);
    cov[7]  = safe(msg->velocity_variance[0]);
    cov[14] = safe(msg->velocity_variance[2]);
    cov[21] = cov[28] = cov[35] = twist_cov_floor_;
  }

  // ------------------------------------------------------------ diagnostics
  void onDiagnostics()
  {
    const double rate = static_cast<double>(rx_count_ - last_rx_) / 5.0;
    RCLCPP_INFO(get_logger(),
                "rx %.1f Hz | published %lu | dropped %lu | resets %lu | "
                "valid(xy=%d z=%d)",
                rate, tx_count_, dropped_, resets_,
                static_cast<int>(xy_valid_), static_cast<int>(z_valid_));
    if (rate < 5.0) {
      RCLCPP_WARN(get_logger(),
                  "odometry rate very low - check MicroXRCEAgent is running, "
                  "the topic is in dds_topics.yaml, and QoS matches");
    }
    last_rx_ = rx_count_;
  }

  // ------------------------------------------------------------------ state
  std::string odom_frame_, base_frame_, footprint_frame_;
  bool publish_tf_{true}, require_valid_{true}, publish_footprint_{true};
  double pose_cov_floor_{1e-4}, twist_cov_floor_{1e-4};

  bool xy_valid_{false}, z_valid_{false};
  bool have_reset_{false};
  uint8_t last_reset_{0};
  uint64_t rx_count_{0}, tx_count_{0}, dropped_{0}, resets_{0}, last_rx_{0};

  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr lpos_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace garudanex

RCLCPP_COMPONENTS_REGISTER_NODE(garudanex::OdometryBridge)
