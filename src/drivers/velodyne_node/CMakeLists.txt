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
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(velodyne_driver REQUIRED)
find_package(udp_driver REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lidar_utils REQUIRED)
find_package(autoware_auto_common REQUIRED)

include_directories(include)

# The library that generates PointCloud2 is separate so that we don't have to lug around unused code
set(CLOUD_LIB velodyne_cloud_node)
add_library(${CLOUD_LIB} SHARED
  include/velodyne_node/velodyne_cloud_node.hpp
  include/velodyne_node/visibility_control.hpp
  src/velodyne_cloud_node.cpp)
autoware_set_compile_options(${CLOUD_LIB})
ament_target_dependencies(${CLOUD_LIB}
  "lidar_utils"
  "rclcpp_lifecycle"
  "velodyne_driver"
  "udp_driver"
  "sensor_msgs"
  "autoware_auto_common")

# generate executable for ros1-style standalone nodes
file(COPY param/vlp16_test.param.yaml DESTINATION "${CMAKE_BINARY_DIR}/param")  #copy the param file into the build directory
set(CLOUD_EXEC "velodyne_cloud_node_exe")
add_executable(${CLOUD_EXEC} src/velodyne_cloud_node_main.cpp)
autoware_set_compile_options(${CLOUD_EXEC})
target_link_libraries(${CLOUD_EXEC} ${CLOUD_LIB})

if(BUILD_TESTING)
    # run common linters
    autoware_static_code_analysis()

    # Test Node
    set(TEST_NODE_NAME "test_listener_exe")
    add_executable(${TEST_NODE_NAME} test/src/test_listener.cpp)
    ament_target_dependencies(${TEST_NODE_NAME} "rclcpp" "sensor_msgs")

    # "Unit" test
    find_package(ament_cmake_gtest REQUIRED)
    find_package(lidar_integration REQUIRED)
    set(VELODYNE_NODE_GTEST velodyne_node_gtest)

    ament_add_gtest(${VELODYNE_NODE_GTEST}
      "test/src/test.cpp"
      "test/src/velodyne_node_test.cpp"
    )

    target_include_directories(${VELODYNE_NODE_GTEST} PRIVATE include)
    target_link_libraries(${VELODYNE_NODE_GTEST} ${CLOUD_LIB} ${BLOCK_LIB})
    ament_target_dependencies(${VELODYNE_NODE_GTEST} "lidar_integration")

    find_package(ros_testing REQUIRED)
    add_ros_test(
      test/velodyne_node.test.py
      TIMEOUT "120"
    )
    add_ros_test(
      test/velodyne_node_bad.test.py
      TIMEOUT "10"
    )
    add_ros_test(
      test/velodyne_node_half_cloud.test.py
      TIMEOUT "120"
    )
endif()

## install stuff
autoware_install(
  LIBRARIES  ${CLOUD_LIB}
  EXECUTABLES ${CLOUD_EXEC}
  HAS_PARAM
)

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
# -Wno-ignored-qualifiers is temp fix for udp_driver bug
target_compile_options(${CLOUD_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-ignored-qualifiers)
target_compile_options(${CLOUD_EXEC} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-ignored-qualifiers)

# Ament Exporting
ament_export_dependencies("sensor_msgs" "velodyne_driver" "udp_driver")
ament_export_interfaces(${BLOCK_EXEC})
ament_package()
