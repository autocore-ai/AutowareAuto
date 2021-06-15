#
# Copyright (c) 2020-2021, Arm Limited and Contributors. All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.5)
project(benchmark_tool_nodes)

### Dependencies

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build

include(ExternalProject)
externalproject_add(perfscript
  URL https://github.com/eclipse-cyclonedds/cyclonedds/archive/refs/tags/0.8.0beta1.tar.gz
  PATCH_COMMAND patch <DOWNLOAD_DIR>/perfscript/examples/perfscript/perftest ${CMAKE_CURRENT_SOURCE_DIR}/perftest.diff
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

externalproject_get_property(perfscript DOWNLOAD_DIR)

### Install

install(
  PROGRAMS
    nodes/benchmark_tool_node.py
    nodes/ros_info_node.py
    nodes/sys_info_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ${DOWNLOAD_DIR}/perfscript/examples/perfscript/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/ros_info_node_launch.test.py
    TIMEOUT "30"
  )

  add_ros_test(
    test/sys_info_node_launch.test.py
    TIMEOUT "30"
  )

  add_ros_test(
    test/benchmark_tool_node_launch.test.py
    TIMEOUT "30"
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE launch/
)
