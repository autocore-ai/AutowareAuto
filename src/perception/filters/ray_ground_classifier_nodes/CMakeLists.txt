# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(ray_ground_classifier_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build cloud node as library
set(CLOUD_NODE_LIB ray_ground_classifier_cloud_node)
ament_auto_add_library(${CLOUD_NODE_LIB} SHARED
  include/ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp
  include/ray_ground_classifier_nodes/visibility_control.hpp
  src/ray_ground_classifier_cloud_node.cpp)
autoware_set_compile_options(${CLOUD_NODE_LIB})

# Associated executable
set(CLOUD_NODE_EXE "${CLOUD_NODE_LIB}_exe")
ament_auto_add_executable(${CLOUD_NODE_EXE} src/ray_ground_classifier_cloud_node_main.cpp)
autoware_set_compile_options(${CLOUD_NODE_EXE})
target_compile_definitions(${CLOUD_NODE_EXE} PRIVATE CMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
target_compile_options(${CLOUD_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion)
target_compile_options(${CLOUD_NODE_EXE} PRIVATE -Wno-sign-conversion -Wno-conversion)

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ray_ground_classifier_nodes_gtest
    test/src/test_ray_ground_classifier_nodes.cpp
  )
  target_include_directories(test_ray_ground_classifier_nodes_gtest
    PRIVATE "test/include" "include"
  )
  target_link_libraries(test_ray_ground_classifier_nodes_gtest
    ${CLOUD_NODE_LIB})
  ament_target_dependencies(test_ray_ground_classifier_nodes_gtest
    "autoware_auto_common"
    "rclcpp"
    "ray_ground_classifier"
    "sensor_msgs"
  )

  add_ros_test(
    test/ray_ground_classifier_fuzzing.test.py
    TIMEOUT "120"
  )
endif()

## install stuff
ament_auto_package(INSTALL_TO_SHARE
  param
  launch)
