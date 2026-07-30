// Copyright 2026 Prince Patel. BSD-3-Clause.
#include <gtest/gtest.h>

#include "garudanex_bridge/frame_transforms.hpp"

using namespace garudanex::frames;  // NOLINT

TEST(FrameTransforms, PositionNedToEnu) {
  // 10 m North, 5 m East, 3 m Down  ->  5 m East, 10 m North, 3 m Up
  const Eigen::Vector3d enu = nedToEnu({10.0, 5.0, -3.0});
  EXPECT_DOUBLE_EQ(enu.x(), 5.0);
  EXPECT_DOUBLE_EQ(enu.y(), 10.0);
  EXPECT_DOUBLE_EQ(enu.z(), 3.0);
}

TEST(FrameTransforms, NedEnuIsInvolution) {
  const Eigen::Vector3d v(1.3, -7.2, 4.4);
  EXPECT_TRUE(enuToNed(nedToEnu(v)).isApprox(v, 1e-12));
}

TEST(FrameTransforms, BodyFrdToFlu) {
  const Eigen::Vector3d flu = frdToFlu({1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(flu.x(), 1.0);
  EXPECT_DOUBLE_EQ(flu.y(), -2.0);
  EXPECT_DOUBLE_EQ(flu.z(), -3.0);
  EXPECT_TRUE(fluToFrd(flu).isApprox(Eigen::Vector3d(1.0, 2.0, 3.0), 1e-12));
}

TEST(FrameTransforms, YawReferenceValues) {
  EXPECT_NEAR(yawNedToEnu(0.0), M_PI_2, 1e-12);        // North -> +90 deg ENU
  EXPECT_NEAR(yawNedToEnu(M_PI_2), 0.0, 1e-12);        // East  ->   0 deg ENU
  EXPECT_NEAR(yawNedToEnu(M_PI), -M_PI_2, 1e-12);      // South -> -90 deg ENU
}

TEST(FrameTransforms, IdentityAttitudeIsNinetyDegYawInEnu) {
  // Identity in NED/FRD means level and pointing North.
  // In ENU/FLU that is a +90 deg yaw (pointing along +Y = North).
  const Eigen::Quaterniond q_enu_flu =
    attitudeNedFrdToEnuFlu(Eigen::Quaterniond::Identity());
  EXPECT_NEAR(std::abs(yawFromQuat(q_enu_flu)), M_PI_2, 1e-9);
}

TEST(FrameTransforms, AttitudeRoundTrip) {
  Eigen::Quaterniond q =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY()));
  q.normalize();
  const Eigen::Quaterniond back =
    attitudeEnuFluToNedFrd(attitudeNedFrdToEnuFlu(q));
  // q and -q are the same rotation, so accept either sign.
  EXPECT_TRUE(back.isApprox(q, 1e-9) ||
              back.coeffs().isApprox(-q.coeffs(), 1e-9));
}

// ---------------------------------------------------------------------------
// Real fixture: captured from GarudaNEX SITL on 2026-07-29 while hovering.
//   ros2 topic echo /fmu/out/vehicle_odometry --qos-reliability best_effort
// ---------------------------------------------------------------------------
TEST(FrameTransforms, RealHoverMessage) {
  const Eigen::Vector3d p_ned(-0.02654953859746456,
                              0.030688496306538582,
                              -2.481778383255005);
  const std::array<float, 4> q_px4 = {
    0.6651347279548645f, 0.001141942571848631f,
    -0.0009458616841584444f, 0.7467219233512878f};

  const Eigen::Vector3d p_enu = nedToEnu(p_ned);

  // The drone was hovering ~2.48 m up. In ENU that must be POSITIVE.
  // Getting this sign backwards is what makes drones descend on command.
  EXPECT_NEAR(p_enu.z(), 2.481778, 1e-6);
  EXPECT_GT(p_enu.z(), 0.0);
  EXPECT_NEAR(p_enu.x(), 0.030688, 1e-6);
  EXPECT_NEAR(p_enu.y(), -0.026550, 1e-6);

  // Cross-check two independent routes to the same answer:
  //   (a) full quaternion composition
  //   (b) the pi/2 - yaw scalar shortcut
  const Eigen::Quaterniond q_ned_frd = fromPx4Quat(q_px4);
  const double yaw_ned = yawFromQuat(q_ned_frd);
  const double yaw_enu_via_quat = yawFromQuat(attitudeNedFrdToEnuFlu(q_ned_frd));
  const double yaw_enu_via_shortcut = yawNedToEnu(yaw_ned);
  EXPECT_NEAR(yaw_enu_via_quat, yaw_enu_via_shortcut, 1e-6);

  // Sanity: PX4 heading was ~96.6 deg (roughly East).
  EXPECT_NEAR(yaw_ned * 180.0 / M_PI, 96.6, 1.0);
}
