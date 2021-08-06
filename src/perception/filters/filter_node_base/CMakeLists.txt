# Copyright 2021 Tier IV, Inc.
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

project(filter_node_base)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/filter_node_base.cpp
)
autoware_set_compile_options(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC
  ${PCL_INCLUDE_DIRS}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  find_package(ament_cmake_gtest REQUIRED)
  set(TEST_SOURCES test/test_filter_node_base.cpp)
  set(TEST_FILTER_NODE_BASE_EXE test_filter_node_base)
  ament_add_gmock(${TEST_FILTER_NODE_BASE_EXE} ${TEST_SOURCES})
  target_compile_options(${TEST_FILTER_NODE_BASE_EXE} PUBLIC "-Wno-pedantic")
  target_link_libraries(${TEST_FILTER_NODE_BASE_EXE} ${PROJECT_NAME})
  ament_target_dependencies(${TEST_FILTER_NODE_BASE_EXE}
    lidar_utils
    fake_test_node
    point_cloud_msg_wrapper
  )
endif()

# ament package generation and installing
ament_auto_package()
