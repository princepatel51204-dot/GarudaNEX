// Copyright 2026 Prince Patel. BSD-3-Clause.
//
// Coordinate frame conversions between PX4 (aerospace) and ROS 2 (REP-103).
//
//   PX4 world : NED  - X North, Y East,  Z Down
//   ROS world : ENU  - X East,  Y North, Z Up
//   PX4 body  : FRD  - X Fwd,   Y Right, Z Down
//   ROS body  : FLU  - X Fwd,   Y Left,  Z Up
//
// Everything here is header-only, stateless and free of ROS types on purpose:
// that is what makes it unit-testable without spinning a node.
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cmath>

namespace garudanex::frames
{

/// Rotation taking NED to ENU. Equivalent to
///   x_enu = y_ned,  y_enu = x_ned,  z_enu = -z_ned
/// which is a rotation of pi about the axis (sqrt(2)/2, sqrt(2)/2, 0).
/// NOTE this is NOT a plain axis relabelling - it is a reflection composed
/// with a rotation, which is why the naive "swap x and y" version of the
/// quaternion conversion produces a mirrored attitude.
inline const Eigen::Quaterniond & q_ENU_NED()
{
  static const Eigen::Quaterniond q(
    0.0, std::sqrt(2.0) / 2.0, std::sqrt(2.0) / 2.0, 0.0);  // (w, x, y, z)
  return q;
}

/// Rotation taking FLU to FRD: pi about the body X axis.
inline const Eigen::Quaterniond & q_FRD_FLU()
{
  static const Eigen::Quaterniond q(0.0, 1.0, 0.0, 0.0);  // (w, x, y, z)
  return q;
}

/// Position or world-frame velocity, NED -> ENU. Self-inverse.
inline Eigen::Vector3d nedToEnu(const Eigen::Vector3d & v)
{
  return {v.y(), v.x(), -v.z()};
}

/// Position or world-frame velocity, ENU -> NED. Self-inverse.
inline Eigen::Vector3d enuToNed(const Eigen::Vector3d & v)
{
  return {v.y(), v.x(), -v.z()};
}

/// Body-frame vector, FRD -> FLU. Self-inverse.
inline Eigen::Vector3d frdToFlu(const Eigen::Vector3d & v)
{
  return {v.x(), -v.y(), -v.z()};
}

/// Body-frame vector, FLU -> FRD. Self-inverse.
inline Eigen::Vector3d fluToFrd(const Eigen::Vector3d & v)
{
  return {v.x(), -v.y(), -v.z()};
}

/// Attitude quaternion (NED->FRD) -> (ENU->FLU).
/// Pre-multiply by the world change of basis, post-multiply by the body one.
inline Eigen::Quaterniond attitudeNedFrdToEnuFlu(const Eigen::Quaterniond & q_ned_frd)
{
  Eigen::Quaterniond q = q_ENU_NED() * q_ned_frd * q_FRD_FLU();
  q.normalize();
  return q;
}

/// Attitude quaternion (ENU->FLU) -> (NED->FRD).
inline Eigen::Quaterniond attitudeEnuFluToNedFrd(const Eigen::Quaterniond & q_enu_flu)
{
  Eigen::Quaterniond q =
    q_ENU_NED().inverse() * q_enu_flu * q_FRD_FLU().inverse();
  q.normalize();
  return q;
}

/// Yaw shortcut. PX4 measures heading CLOCKWISE from North; ROS measures yaw
/// COUNTER-CLOCKWISE from East. Two changes at once - a 90 deg shift of the
/// zero reference AND a sign flip - both encoded by pi/2 - yaw.
/// This single line is responsible for more "my drone flies the wrong way"
/// bugs than anything else in aerial robotics.
inline double yawNedToEnu(double yaw_ned) {return M_PI_2 - yaw_ned;}
inline double yawEnuToNed(double yaw_enu) {return M_PI_2 - yaw_enu;}

/// Extract yaw (rotation about Z) from a quaternion.
inline double yawFromQuat(const Eigen::Quaterniond & q)
{
  return std::atan2(
    2.0 * (q.w() * q.z() + q.x() * q.y()),
    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// PX4 lays quaternions out as (w, x, y, z) - Hamiltonian convention.
inline Eigen::Quaterniond fromPx4Quat(const std::array<float, 4> & q)
{
  return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
}

inline std::array<float, 4> toPx4Quat(const Eigen::Quaterniond & q)
{
  return {static_cast<float>(q.w()), static_cast<float>(q.x()),
    static_cast<float>(q.y()), static_cast<float>(q.z())};
}

}  // namespace garudanex::frames
