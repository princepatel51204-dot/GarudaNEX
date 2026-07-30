cmake_minimum_required(VERSION 3.8)
project(garudanex_description)

find_package(ament_cmake REQUIRED)

# This package ships no code. Everything is a runtime asset, so the
# install rules below are the entire point of this file.
install(
  DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
