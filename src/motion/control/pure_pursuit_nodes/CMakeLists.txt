# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(pure_pursuit_nodes)

## dependencies
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/pure_pursuit_nodes/pure_pursuit_node.hpp
  include/pure_pursuit_nodes/visibility_control.hpp
  src/pure_pursuit_node.cpp
)
# autoware_set_compile_options(${PROJECT_NAME})

set(PURE_PURSUIT_NODE_EXE pure_pursuit_node_exe)
ament_auto_add_executable(${PURE_PURSUIT_NODE_EXE} src/pure_pursuit_node_main.cpp)
# autoware_set_compile_options(${PURE_PURSUIT_NODE_EXE})

if(BUILD_TESTING)
  # run common linters
  # find_package(ament_lint_auto)
  # ament_lint_auto_find_test_dependencies()
  autoware_static_code_analysis()

  # find_package(tf2_msgs REQUIRED)
  # find_package(rclcpp REQUIRED)
  # set(PURE_PURSUIT_NODE_TEST pure_pursuit_integration_test_exe)
  # ament_auto_add_executable(${PURE_PURSUIT_NODE_TEST} test/src/pure_pursuit_integration_test.cpp)
  # autoware_set_compile_options(${PURE_PURSUIT_NODE_TEST})

  # find_package(ros_testing)
  # add_ros_test(test/src/pure_pursuit_node.test.py)
endif()

## install stuff
ament_auto_package(
  INSTALL_TO_SHARE param/
)
