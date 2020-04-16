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

### Build the nodes
project(lidar_integration)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Task is deprecated
add_compile_options(-Wno-deprecated-declarations)

# spoofers
set(SPOOFER_LIB lidar_integration_spoofer)
add_library(${SPOOFER_LIB} SHARED
  src/udp_sender.cpp
  src/vlp16_integration_spoofer.cpp
  src/point_cloud_mutation_spoofer.cpp)
target_include_directories(${SPOOFER_LIB} PUBLIC include)
autoware_set_compile_options(${SPOOFER_LIB})
# FIXME "htons" used in src/udp_sender.cpp produces sign conversion for gcc 7.5
# on ARM 64 Linux when compiling with optimizations, thus disable warning for
# now until UdpSender is ported to cross platform implementation.
# See:
#   https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/merge_requests/116#note_313794141
target_compile_options(${SPOOFER_LIB} PRIVATE -Wno-sign-conversion)
ament_target_dependencies(${SPOOFER_LIB} "rclcpp" "sensor_msgs" "autoware_auto_common")

set(VLP16_INTEGRATION_SPOOFER vlp16_integration_spoofer_exe)
add_executable(${VLP16_INTEGRATION_SPOOFER}
  src/vlp16_integration_spoofer_main.cpp)
autoware_set_compile_options(${VLP16_INTEGRATION_SPOOFER})
target_link_libraries(${VLP16_INTEGRATION_SPOOFER} ${SPOOFER_LIB})
add_dependencies(${VLP16_INTEGRATION_SPOOFER} ${SPOOFER_LIB})

set(POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER point_cloud_mutation_spoofer_exe)
add_executable(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER}
  src/point_cloud_mutation_spoofer_main.cpp
  include/lidar_integration/point_cloud_mutation_spoofer.hpp)
target_include_directories(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER} PUBLIC include)
autoware_set_compile_options(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER})
ament_target_dependencies(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER} "rclcpp")
target_link_libraries(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER} ${SPOOFER_LIB})
add_dependencies(${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER} ${SPOOFER_LIB})

# # LIDAR_INTEGRATION_LISTENER
set(LIDAR_LISTENER lidar_integration_listener)
add_library(${LIDAR_LISTENER} SHARED
  src/lidar_integration_listener.cpp)
target_include_directories(${LIDAR_LISTENER} PUBLIC include)
autoware_set_compile_options(${LIDAR_LISTENER})
ament_target_dependencies(${LIDAR_LISTENER}
  "rclcpp" "sensor_msgs" "autoware_auto_msgs" "autoware_auto_common"
  "rclcpp_lifecycle" "lifecycle_msgs" "std_msgs")

set(LIDAR_INTEGRATION_LISTENER lidar_integration_listener_exe)
add_executable(${LIDAR_INTEGRATION_LISTENER}
  src/lidar_lc_integration_listener.hpp
  src/lidar_integration_listener_main.cpp)
autoware_set_compile_options(${LIDAR_INTEGRATION_LISTENER})
ament_target_dependencies(${LIDAR_INTEGRATION_LISTENER}
  "rclcpp" "sensor_msgs" "autoware_auto_msgs" "autoware_auto_common"
  "rclcpp_lifecycle" "lifecycle_msgs" "std_msgs")
target_link_libraries(${LIDAR_INTEGRATION_LISTENER} ${LIDAR_LISTENER})
add_dependencies(${LIDAR_INTEGRATION_LISTENER} ${LIDAR_LISTENER})

# "Unit test" library
add_library(${PROJECT_NAME} SHARED src/lidar_integration.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} "rclcpp" "rclcpp" "sensor_msgs"
  "autoware_auto_msgs" "autoware_auto_common" "rclcpp_lifecycle"
  "lifecycle_msgs" "std_msgs")
target_link_libraries(${PROJECT_NAME} ${SPOOFER_LIB} ${LIDAR_LISTENER})
add_dependencies(${PROJECT_NAME} ${SPOOFER_LIB} ${LIDAR_LISTENER})

## Testing
if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
endif()

# Ament Exporting
ament_export_dependencies("sensor_msgs" "rclcpp" "autoware_auto_msgs" "autoware_auto_common")

# Python helpers for launch_testing/ros_testing
ament_python_install_package(lidar_integration
    PACKAGE_DIR lidar_integration
)

## install stuff
# Headers
ament_export_include_directories("include")
install(DIRECTORY include/ DESTINATION include)

# Executables
set(_executables "")
list(APPEND _executables
  ${VLP16_INTEGRATION_SPOOFER}
  ${LIDAR_INTEGRATION_LISTENER}
  ${POINT_CLOUD_MUTATION_INTEGRATION_SPOOFER}
)
foreach(_exe ${_executables})
  install(
    TARGETS ${_exe}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endforeach()

# Libraries
set(_libraries "")
list(APPEND _libraries
  ${SPOOFER_LIB}
  ${LIDAR_LISTENER}
  ${PROJECT_NAME}
)
install(
  TARGETS ${_libraries}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(${_libraries})

# CMake Files
install(
  DIRECTORY cmake
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package(
  CONFIG_EXTRAS "lidar_integration-extras.cmake"
)
