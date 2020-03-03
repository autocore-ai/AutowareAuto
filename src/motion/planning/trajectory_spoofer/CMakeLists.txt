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

find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

set(TRAJECTORY_SPOOFER_LIB_SRC
  src/trajectory_spoofer.cpp
)

set(TRAJECTORY_SPOOFER_LIB_HEADERS
  include/trajectory_spoofer/trajectory_spoofer.hpp
  include/trajectory_spoofer/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${TRAJECTORY_SPOOFER_LIB_SRC}
  ${TRAJECTORY_SPOOFER_LIB_HEADERS}
)

set(TRAJECTORY_SPOOFER_EXE_SRC
  src/main.cpp
  src/trajectory_spoofer_node.cpp
)

set(TRAJECTORY_SPOOFER_EXE_HEADERS
  include/trajectory_spoofer/trajectory_spoofer_node.hpp
)

# generate executable for ros1-style standalone nodes
set(TRAJECTORY_SPOOFER_EXE "trajectory_spoofer_exe")
ament_auto_add_executable(${TRAJECTORY_SPOOFER_EXE}
  ${TRAJECTORY_SPOOFER_EXE_SRC}
  ${TRAJECTORY_SPOOFER_EXE_HEADERS}
)
autoware_set_compile_options(${TRAJECTORY_SPOOFER_EXE})

# Testing
if(BUILD_TESTING)
  # Unit tests
  find_package(ament_cmake_gtest)
  set(TEST_SOURCES test/gtest_main.cpp test/test_trajectory_spoofer.cpp)
  set(TEST_TRAJECTORY_SPOOFER_EXE test_trajectory_spoofer)
  ament_add_gtest(${TEST_TRAJECTORY_SPOOFER_EXE} ${TEST_SOURCES})
  ament_target_dependencies(${TEST_TRAJECTORY_SPOOFER_EXE}
    "autoware_auto_common"
    "autoware_auto_msgs"
    "time_utils"
  )
  target_link_libraries(${TEST_TRAJECTORY_SPOOFER_EXE} ${PROJECT_NAME})

  # Static code analyzers
  autoware_static_code_analysis()

  #find_package(autoware_integration_tests)
  #autoware_integration_tests(
  #  EXPECTED_OUTPUT_DIR "test/"
  #  COMMANDS
  #  "${TRAJECTORY_SPOOFER_EXE}"
  #)
endif()

# Install stuff
autoware_install(
  HAS_INCLUDE
  LIBRARIES "${PROJECT_NAME}"
  EXECUTABLES "${TRAJECTORY_SPOOFER_EXE}"
)

# Ament Exporting
ament_export_dependencies("rclcpp")
ament_package()
