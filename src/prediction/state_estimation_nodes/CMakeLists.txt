# Copyright 2021 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(state_estimation_nodes)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(state_estimation_node SHARED
  include/state_estimation_nodes/history.hpp
  include/state_estimation_nodes/measurement_conversion.hpp
  include/state_estimation_nodes/kalman_filter_wrapper.hpp
  include/state_estimation_nodes/state_estimation_node.hpp
  include/state_estimation_nodes/steady_time_grid.hpp
  src/measurement_conversion.cpp
  src/kalman_filter_wrapper.cpp
  src/state_estimation_node.cpp)
# TODO(igor): Remove this once tf2 warnings are removed.
target_include_directories(state_estimation_node SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
target_link_libraries(state_estimation_node Eigen3::Eigen)
autoware_set_compile_options(state_estimation_node)

rclcpp_components_register_node(state_estimation_node
  PLUGIN "autoware::prediction::state_estimation_nodes::StateEstimationNode"
  EXECUTABLE state_estimation_node_exe
)

## Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gmock(test_state_estimation_node
          test/test_history.cpp
          test/test_steady_time_grid.cpp
          test/test_measurement_conversion.cpp
          test/test_kalman_filter_wrapper.cpp
          test/test_state_estimation_node.cpp
  )
  # do not use strict compiler warnings because of GMOCK errors
  # autoware_set_compile_options(test_state_estimation_node)
  # target_compile_options(test_state_estimation_node PRIVATE -Wno-float-conversion -Wno-sign-conversion -Wno-conversion)
  add_dependencies(test_state_estimation_node state_estimation_node)
  target_link_libraries(test_state_estimation_node state_estimation_node)

  find_package(ros_testing REQUIRED)
  add_ros_test(
    test/state_estimation_node_bad.test.py
    TIMEOUT "10"
  )
  add_ros_test(
    test/state_estimation_node_launch.test.py
    TIMEOUT "30"
  )
endif()

ament_auto_package(INSTALL_TO_SHARE param)
