# Copyright 2020, The Autoware Foundation
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

project(trajectory_spoofer)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# generate library
set(TRAJECTORY_SPOOFER_LIB "trajectory_spoofer")
ament_auto_add_library(${TRAJECTORY_SPOOFER_LIB} SHARED
  src/trajectory_spoofer.cpp
  include/trajectory_spoofer/trajectory_spoofer.hpp
)
autoware_set_compile_options(${TRAJECTORY_SPOOFER_LIB})
target_compile_options(${TRAJECTORY_SPOOFER_LIB} PRIVATE
  -Wno-unused-parameter # for speed_ramp_on
)

# generate library
set(TRAJECTORY_SPOOFER_NODE_LIB "trajectory_spoofer_node")
ament_auto_add_library(${TRAJECTORY_SPOOFER_NODE_LIB} SHARED
  src/trajectory_spoofer_node.cpp
  include/trajectory_spoofer/trajectory_spoofer_node.hpp
)
target_link_libraries(${TRAJECTORY_SPOOFER_NODE_LIB} ${TRAJECTORY_SPOOFER_LIB})
autoware_set_compile_options(${TRAJECTORY_SPOOFER_NODE_LIB})

set(NODE_NAME ${TRAJECTORY_SPOOFER_NODE_LIB}_exe)
rclcpp_components_register_node(${TRAJECTORY_SPOOFER_NODE_LIB}
  PLUGIN "autoware::trajectory_spoofer::TrajectorySpooferNode"
  EXECUTABLE ${NODE_NAME}
)

# Testing
if(BUILD_TESTING)
  # Static code analyzers
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})

  # Unit tests
  set(TEST_SOURCES test/gtest_main.cpp test/test_trajectory_spoofer.cpp)
  set(TEST_TRAJECTORY_SPOOFER_EXE test_trajectory_spoofer)
  ament_add_gtest(${TEST_TRAJECTORY_SPOOFER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_TRAJECTORY_SPOOFER_EXE})
  target_compile_options(${TEST_TRAJECTORY_SPOOFER_EXE} PRIVATE -Wno-double-promotion -Wno-float-conversion)
  ament_target_dependencies(${TEST_TRAJECTORY_SPOOFER_EXE}
    "autoware_auto_common"
    "autoware_auto_msgs"
    "time_utils"
  )
  target_link_libraries(
    ${TEST_TRAJECTORY_SPOOFER_EXE}
    ${TRAJECTORY_SPOOFER_LIB}
    ${TRAJECTORY_SPOOFER_NODE_LIB})
  target_include_directories(${TEST_TRAJECTORY_SPOOFER_EXE} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

endif()

# Ament Exporting
ament_auto_package(INSTALL_TO_SHARE launch)
