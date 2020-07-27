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
project(xsens_node)

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

find_package(Boost REQUIRED COMPONENTS system)

set(XSENS_IMU_NODE_LIB xsens_imu_node)
ament_auto_add_library(${XSENS_IMU_NODE_LIB} SHARED
  include/xsens_node/xsens_common_node.hpp
  include/xsens_node/visibility_control.hpp
  src/xsens_imu_node.cpp)
autoware_set_compile_options(${XSENS_IMU_NODE_LIB})
target_link_libraries(${XSENS_IMU_NODE_LIB} ${Boost_LIBRARIES})

set(XSENS_GPS_NODE_LIB xsens_gps_node)
ament_auto_add_library(${XSENS_GPS_NODE_LIB} SHARED
  include/xsens_node/xsens_common_node.hpp
  include/xsens_node/visibility_control.hpp
  src/xsens_gps_node.cpp)
autoware_set_compile_options(${XSENS_GPS_NODE_LIB})
target_link_libraries(${XSENS_GPS_NODE_LIB} ${Boost_LIBRARIES})

# generate executable for ros1-style standalone nodes
set(XSENS_IMU_NODE_EXE "xsens_imu_node_exe")
ament_auto_add_executable(${XSENS_IMU_NODE_EXE} src/xsens_imu_node_main.cpp)
autoware_set_compile_options(${XSENS_IMU_NODE_EXE})
target_link_libraries(${XSENS_IMU_NODE_EXE} ${XSENS_IMU_NODE_LIB})

set(XSENS_GPS_NODE_EXE "xsens_gps_node_exe")
ament_auto_add_executable(${XSENS_GPS_NODE_EXE} src/xsens_gps_node_main.cpp)
autoware_set_compile_options(${XSENS_GPS_NODE_EXE})
target_link_libraries(${XSENS_GPS_NODE_EXE} ${XSENS_GPS_NODE_LIB})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

## install stuff
ament_auto_package(INSTALL_TO_SHARE param)

target_compile_options(${XSENS_IMU_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion)
target_compile_options(${XSENS_IMU_NODE_EXE} PRIVATE -Wno-sign-conversion -Wno-conversion)
target_compile_options(${XSENS_GPS_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion)
target_compile_options(${XSENS_GPS_NODE_EXE} PRIVATE -Wno-sign-conversion -Wno-conversion)
