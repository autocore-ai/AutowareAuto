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

project(hello_world)

# dependencies
find_package(autoware_cmake REQUIRED)  # Required on all packages!
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# includes
include_directories(include)

set(HELLO_WORLD_LIB_SRC
    src/hello_world.cpp
)

set(HELLO_WORLD_LIB_HEADERS
  include/hello_world/hello_world.hpp
  include/hello_world/visibility_control.hpp
)

# generate library
add_library(${PROJECT_NAME} SHARED ${HELLO_WORLD_LIB_SRC} ${HELLO_WORLD_LIB_HEADERS})
autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} "rclcpp")

set(HELLO_WORLD_EXE_SRC
    src/main.cpp
    src/hello_world_node.cpp
)

set(HELLO_WORLD_EXE_HEADERS
  include/hello_world/hello_world_node.hpp
)

# generate executable for ros1-style standalone nodes
set(HELLO_WORLD_EXE "hello_world_exe")
add_executable(${HELLO_WORLD_EXE} ${HELLO_WORLD_EXE_SRC} ${HELLO_WORLD_EXE_HEADERS})
target_link_libraries(${HELLO_WORLD_EXE} ${PROJECT_NAME})
ament_target_dependencies(${HELLO_WORLD_EXE} ${PROJECT_NAME} "rclcpp")
autoware_set_compile_options(${HELLO_WORLD_EXE})

# Testing
if(BUILD_TESTING)
    # Unit tests
    find_package(ament_cmake_gtest)
    set(TEST_SOURCES test/gtest_main.cpp test/test_hello_world.cpp)
    set(TEST_HELLO_WORLD_EXE test_hello_world)
    ament_add_gtest(${TEST_HELLO_WORLD_EXE} ${TEST_SOURCES})
    target_link_libraries(${TEST_HELLO_WORLD_EXE} ${PROJECT_NAME})

    # Static code analyzers
    autoware_static_code_analysis()

    #find_package(autoware_integration_tests)
    #autoware_integration_tests(
    #  EXPECTED_OUTPUT_DIR "test/"
    #  COMMANDS
    #  "${HELLO_WORLD_EXE}"
    #)
endif()

# Install stuff
autoware_install(
  HAS_INCLUDE
  LIBRARIES "${PROJECT_NAME}"
  EXECUTABLES "${HELLO_WORLD_EXE}"
)

# Ament Exporting
ament_export_dependencies("rclcpp")
ament_package()
