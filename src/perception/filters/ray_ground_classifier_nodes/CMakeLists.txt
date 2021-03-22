# Copyright 2017-2020 Apex.AI, Inc., Arm Limited
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(ray_ground_classifier_nodes)

set(use_OMP FALSE)
if(use_OMP)
  find_package(OpenMP REQUIRED)
  set(OpenMP_LIBS gomp)
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
else()
  set(OpenMP_FLAGS "-Wno-unknown-pragmas")
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

rclcpp_components_register_node(${CLOUD_NODE_LIB}
  PLUGIN "autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode"
  EXECUTABLE ${CLOUD_NODE_LIB}_exe
)

target_link_libraries(${CLOUD_NODE_LIB} ${OpenMP_LIBS})

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
target_compile_options(${CLOUD_NODE_LIB} PRIVATE
  -Wno-sign-conversion
  -Wno-conversion
  ${OpenMP_FLAGS}
)

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_ray_ground_classifier_nodes_gtest
    test/src/test_ray_ground_classifier_nodes.cpp
  )
  autoware_set_compile_options(test_ray_ground_classifier_nodes_gtest)
  target_compile_options(test_ray_ground_classifier_nodes_gtest PRIVATE -Wno-conversion)
  target_include_directories(test_ray_ground_classifier_nodes_gtest
    PRIVATE "test/include" "include"
  )
  target_link_libraries(test_ray_ground_classifier_nodes_gtest
    ${CLOUD_NODE_LIB}
    ${OpenMP_LIBS}
  )
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
  add_ros_test(
    test/ray_ground_classifier_node_launch.test.py
    TIMEOUT "30"
  )
endif()

## install stuff
ament_auto_package(INSTALL_TO_SHARE
  param
  launch)
