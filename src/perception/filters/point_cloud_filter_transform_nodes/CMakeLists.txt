# Copyright 2017-2020 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(point_cloud_filter_transform_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### build node as library
set(FILTER_NODE_LIB point_cloud_filter_transform_nodes)
ament_auto_add_library(${FILTER_NODE_LIB} SHARED
    include/point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp
    include/point_cloud_filter_transform_nodes/visibility_control.hpp
    src/point_cloud_filter_transform_node.cpp)
autoware_set_compile_options(${FILTER_NODE_LIB})
target_compile_options(${FILTER_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion)

## Associated executable
set(CLOUD_NODE_EXE point_cloud_filter_transform_node_exe)
ament_auto_add_executable(${CLOUD_NODE_EXE} src/point_cloud_filter_transform_node_main.cpp)
autoware_set_compile_options(${CLOUD_NODE_EXE})
target_compile_options(${CLOUD_NODE_EXE} PRIVATE -Wno-sign-conversion -Wno-conversion)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
  find_package(ament_cmake_gtest REQUIRED)

  # # Unit tests and single-process integration tests
  # TODO(esteve): disabled for now until the lidar_integration package is merged in
  # find_package(ament_cmake_gtest REQUIRED)
  # find_package(apex_test_tools REQUIRED)
  # find_package(lidar_integration REQUIRED)
  # set(FILTER_TRANSFORM_NODE_GTEST filter_transform_node_gtest)
  # apex_test_tools_add_gtest(${FILTER_TRANSFORM_NODE_GTEST}
  #     "test/test_point_cloud_filter_transform.cpp")
  # target_include_directories(${FILTER_TRANSFORM_NODE_GTEST} PRIVATE include)
  # target_link_libraries(${FILTER_TRANSFORM_NODE_GTEST} ${FILTER_NODE_LIB})
  # ament_target_dependencies(${FILTER_TRANSFORM_NODE_GTEST} "lidar_integration" "velodyne_node")

  # # integration test
  # TODO(esteve): disabled for now until the lidar_integration package is merged in
  # find_package(ros_testing REQUIRED)
  # add_ros_test(test/cloud_mutation.test.py)
  # add_ros_test(test/point_cloud_filter_transform_cloud.test.py)
endif()

ament_auto_package(
    INSTALL_TO_SHARE
    design
    param
)
