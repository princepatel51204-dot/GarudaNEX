// Copyright 2026 Prince Patel. BSD-3-Clause.
//
// PX4 v1.17 versions its uORB messages. Verified against a live SITL run:
//
//   VehicleOdometry       MESSAGE_VERSION = 0  -> /fmu/out/vehicle_odometry
//   VehicleAttitude       MESSAGE_VERSION = 0  -> /fmu/out/vehicle_attitude
//   VehicleStatus         MESSAGE_VERSION = 1  -> /fmu/out/vehicle_status_v1
//   VehicleLocalPosition  MESSAGE_VERSION = 1  -> /fmu/out/vehicle_local_position_v1
//
// Subscribing to the unsuffixed name of a versioned message yields SILENCE,
// not an error - the same failure signature as a QoS mismatch. Deriving the
// suffix from the compiled message class means this code keeps working when
// PX4 bumps a version, instead of silently going deaf.
#pragma once

#include <string>

namespace garudanex::px4
{

template<typename MsgT>
inline std::string topicName(const std::string & base)
{
  if constexpr (MsgT::MESSAGE_VERSION == 0) {
    return base;
  } else {
    return base + "_v" + std::to_string(MsgT::MESSAGE_VERSION);
  }
}

}  // namespace garudanex::px4
