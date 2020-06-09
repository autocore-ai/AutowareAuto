# Copyright 2020 Apex.AI, Inc.
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
project(state_estimation_node)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/state_estimation_node/time.hpp
  include/state_estimation_node/measurement.hpp
  include/state_estimation_node/measurement_conversion.hpp
  include/state_estimation_node/kalman_filter_wrapper.hpp
  include/state_estimation_node/state_estimation_node.hpp
  src/measurement_conversion.cpp
  src/kalman_filter_wrapper.cpp
  src/state_estimation_node.cpp)
# TODO(igor): Remove this once tf2 warnings are removed.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
autoware_set_compile_options(${PROJECT_NAME})
rclcpp_components_register_nodes(${PROJECT_NAME}
  "autoware::prediction::state_estimation_node::StateEstimationNode")

set(STATE_ESTIMATION_NODE_EXE "${PROJECT_NAME}_exe")
ament_auto_add_executable(${STATE_ESTIMATION_NODE_EXE}
  "src/state_estimation_node_main.cpp")
autoware_set_compile_options(${STATE_ESTIMATION_NODE_EXE})

## Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
          test/test_measurement.cpp
          test/test_measurement_conversion.cpp
          test/test_kalman_filter_wrapper.cpp
#          test/test_state_estimation_node.cpp
  )
  # TODO(niosus): Above test is temporarily disabled until #488 is fixed
  add_dependencies(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})

  find_package(ros_testing REQUIRED)
  add_ros_test(
    test/state_estimation_node_bad.test.py
    TIMEOUT "10"
  )
endif()

ament_auto_package(INSTALL_TO_SHARE param)
