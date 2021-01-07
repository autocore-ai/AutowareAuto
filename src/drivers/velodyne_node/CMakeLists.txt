# Copyright 2018 the Autoware Foundation
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

# The library that generates PointCloud2 is separate so that we don't have to lug around unused code
set(CLOUD_LIB velodyne_cloud_node)
ament_auto_add_library(${CLOUD_LIB} SHARED
  include/velodyne_node/velodyne_cloud_node.hpp
  include/velodyne_node/visibility_control.hpp
  src/velodyne_cloud_node.cpp)
autoware_set_compile_options(${CLOUD_LIB})

# generate executable for ros1-style standalone nodes
set(CLOUD_EXEC "velodyne_cloud_node_exe")
ament_auto_add_executable(${CLOUD_EXEC} src/velodyne_cloud_node_main.cpp)
autoware_set_compile_options(${CLOUD_EXEC})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # Test Node
    set(TEST_NODE_NAME "test_listener_exe")
    add_executable(${TEST_NODE_NAME} test/src/test_listener.cpp)
    ament_target_dependencies(${TEST_NODE_NAME} "rclcpp" "sensor_msgs")

    # "Unit" test
    set(VELODYNE_NODE_GTEST velodyne_node_gtest)

    ament_add_gtest(${VELODYNE_NODE_GTEST}
      "test/src/test.cpp"
      "test/src/velodyne_node_test.cpp"
    )

    target_include_directories(${VELODYNE_NODE_GTEST} PRIVATE include)
    target_link_libraries(${VELODYNE_NODE_GTEST} ${CLOUD_LIB} ${BLOCK_LIB})
    ament_target_dependencies(${VELODYNE_NODE_GTEST} "lidar_integration")

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
    add_ros_test(
      test/velodyne_node_launch.test.py
      TIMEOUT "30"
    )
endif()

## install stuff
ament_auto_package(INSTALL_TO_SHARE param launch)

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
# -Wno-ignored-qualifiers is temp fix for udp_driver bug
target_compile_options(${CLOUD_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-ignored-qualifiers)
target_compile_options(${CLOUD_EXEC} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-ignored-qualifiers)
