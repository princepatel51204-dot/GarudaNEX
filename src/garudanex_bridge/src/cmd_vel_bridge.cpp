// Copyright 2026 Prince Patel. BSD-3-Clause.
//
// GarudaNEX cmd_vel bridge: ROS 2 velocity commands -> PX4 offboard setpoints.
//
//   /cmd_vel  (ENU world / FLU body, from Nav2)
//        |
//        +--> /fmu/in/offboard_control_mode   20 Hz, UNCONDITIONAL
//        +--> /fmu/in/trajectory_setpoint     20 Hz, NED
//
// This is the SECOND and last place in GarudaNEX where a coordinate frame
// conversion happens. The first is the odometry bridge.
//
// It does NOT arm, disarm or change flight mode. Those are mission decisions
// and live in garudanex_mission. This node only translates.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include "garudanex_bridge/frame_transforms.hpp"
#include "garudanex_bridge/px4_topics.hpp"

namespace garudanex
{

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleLocalPosition;
namespace gf = garudanex::frames;

class CmdVelBridge : public rclcpp::Node
{
public:
  explicit CmdVelBridge(const rclcpp::NodeOptions & options)
  : Node("garudanex_cmd_vel_bridge", options)
  {
    cruise_alt_    = declare_parameter<double>("cruise_altitude", 2.5);
    max_xy_vel_    = declare_parameter<double>("max_xy_velocity", 1.5);
    max_yaw_rate_  = declare_parameter<double>("max_yaw_rate", 0.8);
    cmd_timeout_   = declare_parameter<double>("cmd_timeout", 0.5);
    setpoint_hz_   = declare_parameter<double>("setpoint_rate", 20.0);
    alt_kp_        = declare_parameter<double>("altitude_kp", 1.0);
    max_z_vel_     = declare_parameter<double>("max_z_velocity", 1.0);
    stamped_       = declare_parameter<bool>("stamped_cmd_vel", false);

    rmw_qos_profile_t profile = rmw_qos_profile_sensor_data;
    auto px4_qos = rclcpp::QoS(
      rclcpp::QoSInitialization(profile.history, 5), profile);

    // Nav2 on Jazzy publishes geometry_msgs/Twist on /cmd_vel by default.
    // Kilted and later default to TwistStamped. Supporting both behind a
    // parameter means the Nav2 upgrade is a config change, not a code change.
    if (stamped_) {
      sub_stamped_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", rclcpp::QoS(10),
        [this](geometry_msgs::msg::TwistStamped::SharedPtr m) {
          acceptCommand(m->twist);
        });
    } else {
      sub_plain_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10),
        [this](geometry_msgs::msg::Twist::SharedPtr m) {
          acceptCommand(*m);
        });
    }

    lpos_sub_ = create_subscription<VehicleLocalPosition>(
      px4::topicName<VehicleLocalPosition>("/fmu/out/vehicle_local_position"),
      px4_qos,
      [this](VehicleLocalPosition::SharedPtr m) {
        heading_ned_ = m->heading;
        z_ned_       = m->z;
        have_state_  = m->xy_valid && m->z_valid;
      });

    ocm_pub_ = create_publisher<OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    sp_pub_  = create_publisher<TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);

    // WALL timer on purpose. This heartbeat must keep ticking at a real-world
    // rate even if sim time stalls, because PX4's offboard timeout is measured
    // against its own clock.
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / setpoint_hz_),
      std::bind(&CmdVelBridge::tick, this));

    diag_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&CmdVelBridge::onDiagnostics, this));

    RCLCPP_INFO(get_logger(), "GarudaNEX cmd_vel bridge up");
    RCLCPP_INFO(get_logger(), "  /cmd_vel type   : %s",
                stamped_ ? "geometry_msgs/TwistStamped"
                         : "geometry_msgs/Twist");
    RCLCPP_INFO(get_logger(), "  cruise altitude : %.2f m AGL", cruise_alt_);
    RCLCPP_INFO(get_logger(), "  limits          : %.2f m/s xy, %.2f rad/s yaw",
                max_xy_vel_, max_yaw_rate_);
    RCLCPP_INFO(get_logger(), "  setpoint rate   : %.1f Hz (PX4 floor is ~2 Hz)",
                setpoint_hz_);
  }

private:
  void acceptCommand(const geometry_msgs::msg::Twist & t)
  {
    last_cmd_ = t;
    last_cmd_time_ = now();
    ++cmd_count_;
  }

  static double clampv(double v, double lim)
  {
    return std::clamp(v, -lim, lim);
  }

  void tick()
  {
    // ---- 1. Heartbeat. ALWAYS. ----
    // PX4 exits offboard mode if OffboardControlMode stops arriving at ~2 Hz.
    // position=true  -> we command Z as a position (altitude hold)
    // velocity=true  -> we command XY as a velocity
    OffboardControlMode ocm{};
    ocm.position     = false;   // MUST be false. See note below.
    ocm.velocity     = true;    // single, unambiguous control level
    ocm.acceleration = false;
    ocm.attitude     = false;
    ocm.body_rate    = false;
    ocm.timestamp    = nowUs();
    ocm_pub_->publish(ocm);

    if (!have_state_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "waiting for a valid PX4 local position before sending setpoints");
      return;
    }

    // ---- 2. Decide the command ----
    double vx_flu = 0.0, vy_flu = 0.0, yaw_rate_enu = 0.0;
    const double age = (now() - last_cmd_time_).seconds();
    const bool fresh = (cmd_count_ > 0) && (age < cmd_timeout_);

    if (fresh) {
      vx_flu       = clampv(last_cmd_.linear.x,  max_xy_vel_);
      vy_flu       = clampv(last_cmd_.linear.y,  max_xy_vel_);
      yaw_rate_enu = clampv(last_cmd_.angular.z, max_yaw_rate_);
    } else if (cmd_count_ > 0) {
      // A stale command must NOT persist. A drone that keeps flying at 1.5 m/s
      // because its planner died is a drone that hits a wall.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "/cmd_vel stale (%.2f s) - holding position", age);
      ++stale_ticks_;
    }

    // ---- 3. Body FLU velocity -> world NED velocity ----
    // Nav2 emits velocity in the ROBOT BODY frame. Rotate it into the world
    // using the current heading, then convert ENU -> NED.
    //   yaw_enu = pi/2 - yaw_ned   (Appendix C)
    const double yaw_enu = gf::yawNedToEnu(heading_ned_);
    const double c = std::cos(yaw_enu), s = std::sin(yaw_enu);
    const double vx_enu = vx_flu * c - vy_flu * s;
    const double vy_enu = vx_flu * s + vy_flu * c;
    const double vx_ned = vy_enu;   // NED x (North) <- ENU y (North)
    const double vy_ned = vx_enu;   // NED y (East)  <- ENU x (East)

    // ---- 4. Publish ----
    // Altitude hold as a VELOCITY on the Z axis, not a position setpoint.
    //   current_alt (AGL, +up) = -z_ned
    //   NED z velocity is POSITIVE DOWN, so climbing needs a negative value.
    const double current_alt = -z_ned_;
    const double alt_err     = cruise_alt_ - current_alt;   // + means climb
    const double vz_ned      = -clampv(alt_kp_ * alt_err, max_z_vel_);

    TrajectorySetpoint sp{};
    // All three axes commanded as velocity. Nothing is NaN, nothing is mixed.
    sp.position = {NAN, NAN, NAN};
    sp.velocity = {static_cast<float>(vx_ned), static_cast<float>(vy_ned),
                   static_cast<float>(vz_ned)};
    sp.acceleration = {NAN, NAN, NAN};
    sp.jerk         = {NAN, NAN, NAN};
    sp.yaw      = NAN;
    // NED yaw increases CLOCKWISE, ENU yaw increases COUNTER-CLOCKWISE.
    // Miss this negation and the drone rotates away from every goal.
    sp.yawspeed = static_cast<float>(-yaw_rate_enu);
    sp.timestamp = nowUs();
    sp_pub_->publish(sp);
    ++tx_count_;
  }

  uint64_t nowUs() const
  {
    // PX4 timestamps are MICROSECONDS. Nanoseconds are silently rejected.
    return static_cast<uint64_t>(now().nanoseconds() / 1000);
  }

  void onDiagnostics()
  {
    RCLCPP_INFO(get_logger(),
      "setpoints %lu | cmd_vel rx %lu | stale %lu | alt %.2f / %.2f m | "
      "heading_ned %.1f deg | state %s",
      tx_count_, cmd_count_, stale_ticks_, -z_ned_, cruise_alt_,
      heading_ned_ * 180.0 / M_PI, have_state_ ? "ok" : "INVALID");
  }

  double cruise_alt_, max_xy_vel_, max_yaw_rate_, cmd_timeout_, setpoint_hz_;
  double alt_kp_{1.0}, max_z_vel_{1.0};
  bool stamped_{false};

  double heading_ned_{0.0}, z_ned_{0.0};
  bool have_state_{false};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  uint64_t tx_count_{0}, cmd_count_{0}, stale_ticks_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_plain_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_stamped_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr lpos_sub_;
  rclcpp::Publisher<OffboardControlMode>::SharedPtr ocm_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr sp_pub_;
  rclcpp::TimerBase::SharedPtr timer_, diag_timer_;
};

}  // namespace garudanex

RCLCPP_COMPONENTS_REGISTER_NODE(garudanex::CmdVelBridge)
