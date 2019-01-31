# Copyright 2018 Apex.AI, Inc.
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
project(velodyne_node)

## dependencies
find_package(autoware_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(velodyne_driver REQUIRED)
find_package(udp_driver REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options filesystem)
find_package(rclcpp_lifecycle REQUIRED)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif()

include_directories(include)

# The library that generates PointCloud2 is separate so that we don't have to lug around unused code
set(CLOUD_LIB velodyne_cloud_node)
add_library(${CLOUD_LIB} SHARED
  include/velodyne_node/velodyne_cloud_node.hpp
  include/velodyne_node/visibility_control.hpp
  src/velodyne_cloud_node.cpp)
autoware_set_compile_options(${CLOUD_LIB})
ament_target_dependencies(${CLOUD_LIB}
  "rclcpp_lifecycle"
  "velodyne_driver"
  "udp_driver"
  "sensor_msgs")
target_link_libraries(${CLOUD_LIB} ${Boost_LIBRARIES})

# generate executable for ros1-style standalone nodes
file(COPY param/vlp16_test.param.yaml DESTINATION "${CMAKE_BINARY_DIR}/param")  #copy the param file into the build directory
set(CLOUD_EXEC "velodyne_cloud_node_exe")
add_executable(${CLOUD_EXEC} src/velodyne_cloud_node_main.cpp)
autoware_set_compile_options(${CLOUD_EXEC})
target_link_libraries(${CLOUD_EXEC} ${CLOUD_LIB})
ament_target_dependencies(${CLOUD_EXEC} "Boost")

if(BUILD_TESTING)
    autoware_static_code_analysis()
endif()

## install stuff
autoware_install(
  LIBRARIES  ${CLOUD_LIB}
  EXECUTABLES ${CLOUD_EXEC}
)
# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
target_compile_options(${CLOUD_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion)
target_compile_options(${CLOUD_EXEC} PRIVATE -Wno-sign-conversion -Wno-conversion)

# Ament Exporting
ament_export_dependencies("sensor_msgs" "velodyne_driver" "udp_driver")
ament_export_interfaces(${BLOCK_EXEC})
ament_package()
