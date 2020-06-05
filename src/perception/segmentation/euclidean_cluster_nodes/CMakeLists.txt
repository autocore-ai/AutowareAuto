# Copyright 2017-2019 Apex.AI, Inc.
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

### Build the nodes
project(euclidean_cluster_nodes)

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

# Library
set(CLUSTER_NODE_LIB euclidean_cluster_node)
ament_auto_add_library(${CLUSTER_NODE_LIB} SHARED
  include/euclidean_cluster_nodes/details/common.hpp
  include/euclidean_cluster_nodes/euclidean_cluster_node.hpp
  src/common.cpp
  src/euclidean_cluster_node.cpp)
autoware_set_compile_options(${CLUSTER_NODE_LIB})
target_compile_options(
  ${CLUSTER_NODE_LIB} PUBLIC "-Wno-sign-conversion" "-Wno-conversion")

# Executable
set(CLUSTER_MAIN euclidean_cluster_exe)
ament_auto_add_executable(${CLUSTER_MAIN} src/euclidean_cluster_main.cpp)
autoware_set_compile_options(${CLUSTER_MAIN})

## Testing
if(BUILD_TESTING)
  # Static checking only if built via ament
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
# Ament Exporting
ament_auto_package(INSTALL_TO_SHARE
  launch
  param)
