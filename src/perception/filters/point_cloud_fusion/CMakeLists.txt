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
project(point_cloud_fusion)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(lidar_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIR}
)

# includes
include_directories(include)
set(PC_FUSION_LIB pointcloud_fusion_node)

add_library(${PC_FUSION_LIB} SHARED
  include/point_cloud_fusion/point_cloud_fusion.hpp
  src/point_cloud_fusion.cpp
  include/point_cloud_fusion/point_cloud_transform.hpp include/point_cloud_fusion/visibility_control.hpp)

autoware_set_compile_options(${PC_FUSION_LIB})
ament_target_dependencies(${PC_FUSION_LIB}
  "lidar_utils"
  "sensor_msgs"
  "rclcpp"
  "rclcpp_lifecycle"
  "message_filters"
  "tf2_ros"
  "tf2_geometry_msgs"
)
#copy the param file into the build directory
file(COPY param/pc_fusion.example.param.yaml DESTINATION "${CMAKE_BINARY_DIR}/param")

set(PC_FUSION_EXEC "pointcloud_fusion_node_exe")
add_executable(${PC_FUSION_EXEC} src/point_cloud_fusion_node_main.cpp)
autoware_set_compile_options(${PC_FUSION_EXEC})
target_link_libraries(${PC_FUSION_EXEC} ${PC_FUSION_LIB})
ament_target_dependencies(${PC_FUSION_EXEC})

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
  find_package(ament_cmake_gtest REQUIRED)

  # gtest
  set(PCF_GTEST pointcloud_fusion_gtest)
  ament_add_gtest(${PCF_GTEST}
          test/test_point_cloud_fusion.hpp
          test/test_point_cloud_fusion.cpp)
  target_link_libraries(${PCF_GTEST} ${PC_FUSION_LIB})
  target_include_directories(${PCF_GTEST} PRIVATE "test/include" "include")
  ament_target_dependencies(${PCF_GTEST} "rclcpp")

  target_compile_options(${PCF_GTEST} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)
endif()

# required by tf2, message_filters
target_compile_options(${PC_FUSION_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)
target_compile_options(${PC_FUSION_EXEC} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

autoware_install(LIBRARIES ${PC_FUSION_LIB}
          EXECUTABLES ${PC_FUSION_EXEC}
)
# Ament Exporting
ament_export_dependencies("sensor_msgs" "rclcpp" "rclcpp_lifecycle" "message_filters" "tf2_ros" "tf2_geometry_msgs")
ament_package()
