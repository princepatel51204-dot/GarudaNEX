cmake_minimum_required(VERSION 3.8)
project(garudanex_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------- odometry bridge ----------------
add_library(odometry_bridge_component SHARED src/odometry_bridge.cpp)
target_include_directories(odometry_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(odometry_bridge_component Eigen3::Eigen)
ament_target_dependencies(odometry_bridge_component
  rclcpp rclcpp_components px4_msgs nav_msgs geometry_msgs
  tf2 tf2_ros tf2_geometry_msgs)
rclcpp_components_register_node(odometry_bridge_component
  PLUGIN "garudanex::OdometryBridge"
  EXECUTABLE odometry_bridge_node)

# ---------------- scan frame relay ----------------
add_library(scan_frame_relay_component SHARED src/scan_frame_relay.cpp)
target_include_directories(scan_frame_relay_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(scan_frame_relay_component
  rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(scan_frame_relay_component
  PLUGIN "garudanex::ScanFrameRelay"
  EXECUTABLE scan_frame_relay_node)

# ---------------- cmd_vel bridge ----------------
add_library(cmd_vel_bridge_component SHARED src/cmd_vel_bridge.cpp)
target_include_directories(cmd_vel_bridge_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(cmd_vel_bridge_component Eigen3::Eigen)
ament_target_dependencies(cmd_vel_bridge_component
  rclcpp rclcpp_components px4_msgs geometry_msgs)
rclcpp_components_register_node(cmd_vel_bridge_component
  PLUGIN "garudanex::CmdVelBridge"
  EXECUTABLE cmd_vel_bridge_node)

install(TARGETS
  odometry_bridge_component
  scan_frame_relay_component
  cmd_vel_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_transforms test/test_frame_transforms.cpp)
  target_include_directories(test_frame_transforms PRIVATE include)
  target_link_libraries(test_frame_transforms Eigen3::Eigen)
endif()

ament_export_include_directories(include)
ament_export_libraries(odometry_bridge_component scan_frame_relay_component cmd_vel_bridge_component)
ament_export_dependencies(rclcpp px4_msgs nav_msgs geometry_msgs sensor_msgs
                          tf2_ros Eigen3)
ament_package()
