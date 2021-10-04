# Copyright 2020 The Autoware Foundation
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

project(behavior_planner_nodes)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(BEHAVIOR_PLANNER_NODE_SRC
  src/behavior_planner_node.cpp
)

set(BEHAVIOR_PLANNER_NODE_HEADERS
  include/behavior_planner_nodes/behavior_planner_node.hpp
  include/behavior_planner_nodes/visibility_control.hpp
)

# generate component node library
ament_auto_add_library(behavior_planner_node SHARED
  ${BEHAVIOR_PLANNER_NODE_SRC}
  ${BEHAVIOR_PLANNER_NODE_HEADERS}
)
set(NODE_NAME behavior_planner_node_exe)
autoware_set_compile_options(behavior_planner_node)
rclcpp_components_register_node(behavior_planner_node
  PLUGIN "autoware::behavior_planner_nodes::BehaviorPlannerNode"
  EXECUTABLE ${NODE_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})

  # Unit tests
  set(TEST_SOURCES test/test_behavior_planner_node.cpp)
  set(TEST_BEHAVIOR_PLANNER_NODE_EXE test_behavior_planner_node)
  ament_add_gtest(${TEST_BEHAVIOR_PLANNER_NODE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_BEHAVIOR_PLANNER_NODE_EXE})
  target_link_libraries(${TEST_BEHAVIOR_PLANNER_NODE_EXE} behavior_planner_node)
endif()

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
    param
)
