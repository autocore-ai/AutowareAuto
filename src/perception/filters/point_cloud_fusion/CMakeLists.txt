# Copyright 2017-2018 Apex.AI, Inc.
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

cmake_minimum_required(VERSION 3.5)

### Export headers
project(point_cloud_fusion)

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

set(PC_FUSION_LIB pointcloud_fusion_node)
ament_auto_add_library(${PC_FUSION_LIB} SHARED
  include/point_cloud_fusion/point_cloud_fusion.hpp
  src/point_cloud_fusion.cpp
  include/point_cloud_fusion/visibility_control.hpp)
autoware_set_compile_options(${PC_FUSION_LIB})

set(PC_FUSION_EXEC pointcloud_fusion_node_exe)
ament_auto_add_executable(${PC_FUSION_EXEC} src/point_cloud_fusion_node_main.cpp)
autoware_set_compile_options(${PC_FUSION_EXEC})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(PCF_GTEST pointcloud_fusion_gtest)
  ament_add_gtest(${PCF_GTEST}
          test/test_point_cloud_fusion.hpp
          test/test_point_cloud_fusion.cpp)
  target_link_libraries(${PCF_GTEST} ${PC_FUSION_LIB})
  target_include_directories(${PCF_GTEST} PRIVATE "test/include" "include")
  ament_target_dependencies(${PCF_GTEST} "rclcpp")

  target_compile_options(${PCF_GTEST} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare)
endif()

# required by tf2, message_filters
target_compile_options(${PC_FUSION_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare)
target_compile_options(${PC_FUSION_EXEC} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare)

# Ament Exporting
ament_auto_package(
    INSTALL_TO_SHARE
    param
)
