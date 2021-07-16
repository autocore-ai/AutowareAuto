# Copyright 2021 The Autoware Foundation
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

project(tracking_nodes)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(MULTI_OBJECT_TRACKER_NODE_SRC
  src/multi_object_tracker_node.cpp
)

set(MULTI_OBJECT_TRACKER_NODE_HEADERS
  include/tracking_nodes/multi_object_tracker_node.hpp
)

# generate component node library
ament_auto_add_library(multi_object_tracker_node SHARED
  ${MULTI_OBJECT_TRACKER_NODE_SRC}
  ${MULTI_OBJECT_TRACKER_NODE_HEADERS}
)
# TODO(nikolai.morin): See #1059
target_include_directories(multi_object_tracker_node SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

autoware_set_compile_options(multi_object_tracker_node)
rclcpp_components_register_node(multi_object_tracker_node
  PLUGIN "autoware::tracking_nodes::MultiObjectTrackerNode"
  EXECUTABLE multi_object_tracker_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/launch.test.py
    TIMEOUT "30"
  )
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
