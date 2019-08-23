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

## dependencies
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(euclidean_cluster REQUIRED)
find_package(autoware_auto_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(voxel_grid_nodes REQUIRED)

# Library
set(CLUSTER_NODE_LIB euclidean_cluster_node)
add_library(${CLUSTER_NODE_LIB} SHARED
  # include/euclidean_cluster_nodes/details/common.hpp
  include/euclidean_cluster_nodes/euclidean_cluster_node.hpp
  # src/common.cpp
  src/euclidean_cluster_node.cpp)
target_include_directories(${CLUSTER_NODE_LIB} PUBLIC "include")
autoware_set_compile_options(${CLUSTER_NODE_LIB})
ament_target_dependencies(${CLUSTER_NODE_LIB}
  "rclcpp"
  "autoware_auto_msgs"
  "euclidean_cluster"
  "autoware_auto_geometry"
  "sensor_msgs"
  "voxel_grid_nodes")

# Executable
set(CLUSTER_MAIN euclidean_cluster_exe)
add_executable(${CLUSTER_MAIN} src/euclidean_cluster_main.cpp)
autoware_set_compile_options(${CLUSTER_MAIN})
target_link_libraries(${CLUSTER_MAIN} ${CLUSTER_NODE_LIB})

## Testing
if(BUILD_TESTING)
  # Static checking only if built via ament
  autoware_static_code_analysis()
endif()
# Ament Exporting
autoware_install(LIBRARIES ${CLUSTER_NODE_LIB}
  EXECUTABLES ${CLUSTER_MAIN}
  HAS_PARAM
  HAS_INCLUDE)
ament_export_dependencies(
  "rclcpp"
  "autoware_auto_msgs"
  "euclidean_cluster"
  "autoware_auto_geometry"
  "sensor_msgs"
  "voxel_grid_nodes")
ament_package()
