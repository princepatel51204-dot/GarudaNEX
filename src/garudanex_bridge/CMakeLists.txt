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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Built as a COMPOSABLE COMPONENT, not a plain executable. This lets the node
# either run standalone or be loaded into a shared container alongside the
# cmd_vel bridge, where intra-process comms pass messages by pointer instead
# of serialising them. Costs nothing to write this way.
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

install(TARGETS odometry_bridge_component
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
ament_export_libraries(odometry_bridge_component)
ament_export_dependencies(rclcpp px4_msgs nav_msgs geometry_msgs tf2_ros Eigen3)
ament_package()
