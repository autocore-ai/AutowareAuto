# Copyright 2018 the Autoware Foundation
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

### Build the nodes
project(velodyne_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# The library that generates PointCloud2 is separate so that we don't have to lug around unused code
set(CLOUD_LIB velodyne_cloud_node)
ament_auto_add_library(${CLOUD_LIB} SHARED
  include/velodyne_nodes/velodyne_cloud_node.hpp
  include/velodyne_nodes/visibility_control.hpp
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
    autoware_set_compile_options(${TEST_NODE_NAME})
    target_compile_options(${TEST_NODE_NAME} PRIVATE -Wno-conversion)
    ament_target_dependencies(${TEST_NODE_NAME} "rclcpp" "sensor_msgs")

    # "Unit" test
    set(VELODYNE_NODE_GTEST velodyne_node_gtest)

    ament_add_gtest(${VELODYNE_NODE_GTEST}
      "test/src/test.cpp"
      "test/src/velodyne_node_test.cpp"
    )

    autoware_set_compile_options(${VELODYNE_NODE_GTEST})
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

    find_package(autoware_testing REQUIRED)
    add_smoke_test(${PROJECT_NAME} ${CLOUD_EXEC}
      EXECUTABLE_ARGUMENTS "--model vlp16"
    )
endif()

## install stuff
ament_auto_package(INSTALL_TO_SHARE param launch)
