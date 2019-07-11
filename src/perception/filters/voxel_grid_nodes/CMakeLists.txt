# Copyright 2017-2018 Apex.AI, Inc.
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
project(voxel_grid_nodes)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(lidar_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(voxel_grid REQUIRED)

# includes
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  include/voxel_grid_nodes/algorithm/voxel_cloud_base.hpp
  include/voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp
  include/voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp
  include/voxel_grid_nodes/visibility_control.hpp
  src/algorithm/voxel_cloud_base.cpp
  src/algorithm/voxel_cloud_approximate.cpp
  src/algorithm/voxel_cloud_centroid.cpp
  include/voxel_grid_nodes/voxel_cloud_node.hpp
  src/voxel_cloud_node.cpp
)
autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  "lidar_utils"
  "sensor_msgs"
  "voxel_grid"
  "rclcpp"
  "rclcpp_lifecycle"
)

# Cloud executable
set(VOXEL_CLOUD_EXE voxel_grid_cloud_node_exe)
add_executable(${VOXEL_CLOUD_EXE} src/voxel_cloud_main.cpp)
autoware_set_compile_options(${VOXEL_CLOUD_EXE})
target_link_libraries(${VOXEL_CLOUD_EXE} ${PROJECT_NAME})
ament_target_dependencies(${VOXEL_CLOUD_EXE} "rclcpp" "rclcpp_lifecycle")
target_compile_definitions(${VOXEL_CLOUD_EXE} PRIVATE
    CMAKE_INSTALL_PREFIX="${CMAKE_INSTALL_PREFIX}")

file(COPY param/vlp16_lexus_centroid.param.yaml DESTINATION "${CMAKE_BINARY_DIR}/param")  #copy the param file into the build directory

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
  find_package(ament_cmake_gtest REQUIRED)


  # gtest
  set(VOXEL_GRID_NODE_GTEST voxel_algorithm_gtest)
  ament_add_gtest(${VOXEL_GRID_NODE_GTEST} test/test_voxel_grid_nodes.cpp)
  target_link_libraries(${VOXEL_GRID_NODE_GTEST} ${PROJECT_NAME})
  target_include_directories(${VOXEL_GRID_NODE_GTEST} PRIVATE "test/include" "include")
endif()

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion)

autoware_install(LIBRARIES ${PROJECT_NAME} EXECUTABLES ${VOXEL_CLOUD_EXE} HAS_INCLUDE)
# Ament Exporting
ament_export_dependencies("sensor_msgs" "voxel_grid" "rclcpp" "rclcpp_lifecycle")
ament_package()
