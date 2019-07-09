# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(ray_ground_classifier_nodes)

## dependencies
find_package(autoware_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ray_ground_classifier REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

# includes
include_directories(include)

### Build cloud node as library
set(CLOUD_NODE_LIB ray_ground_classifier_cloud_node)
add_library(${CLOUD_NODE_LIB} SHARED
  include/ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp
  include/ray_ground_classifier_nodes/visibility_control.hpp
  src/ray_ground_classifier_cloud_node.cpp)
autoware_set_compile_options(${CLOUD_NODE_LIB})
ament_target_dependencies(${CLOUD_NODE_LIB}
  "sensor_msgs"
  "rclcpp"
  "ray_ground_classifier"
  "rclcpp_lifecycle")
# Associated executable
set(CLOUD_NODE_EXE "${CLOUD_NODE_LIB}_exe")
add_executable(${CLOUD_NODE_EXE} src/ray_ground_classifier_cloud_node_main.cpp)
target_link_libraries(${CLOUD_NODE_EXE} ${CLOUD_NODE_LIB})
ament_target_dependencies(${CLOUD_NODE_EXE})
autoware_set_compile_options(${CLOUD_NODE_EXE})
target_compile_definitions(${CLOUD_NODE_EXE} PRIVATE CMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
target_compile_options(${CLOUD_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion)
target_compile_options(${CLOUD_NODE_EXE} PRIVATE -Wno-sign-conversion -Wno-conversion)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
endif()

## install stuff
autoware_install(
  HAS_PARAM
  HAS_LAUNCH
  LIBRARIES ${CLOUD_NODE_LIB}
  EXECUTABLES ${CLOUD_NODE_EXE})
# Ament Exporting
ament_export_dependencies(
  "ray_ground_classifier"
  "rclcpp_lifecycle")
ament_export_interfaces(${CLOUD_NODE_EXE})
ament_package()

