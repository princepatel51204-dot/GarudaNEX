cmake_minimum_required(VERSION 3.8)
project(garudanex_sim)

find_package(ament_cmake REQUIRED)

# Configuration-only package: worlds, models and bridge YAML.
install(
  DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
