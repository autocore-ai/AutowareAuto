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

project(hello_world)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(HELLO_WORLD_LIB_SRC
  src/hello_world.cpp
)

set(HELLO_WORLD_LIB_HEADERS
  include/hello_world/hello_world.hpp
  include/hello_world/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${HELLO_WORLD_LIB_SRC}
  ${HELLO_WORLD_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(HELLO_WORLD_NODE_SRC
  src/hello_world_node.cpp
)

set(HELLO_WORLD_NODE_HEADERS
  include/hello_world/hello_world_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${HELLO_WORLD_NODE_SRC}
  ${HELLO_WORLD_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_nodes(${PROJECT_NAME}_node "autoware::hello_world::HelloWorldNode")

set(HELLO_WORLD_EXE_SRC
  src/main.cpp
)

# generate stand-alone executable
ament_auto_add_executable(${PROJECT_NAME}_exe
  ${HELLO_WORLD_EXE_SRC}
)
autoware_set_compile_options(${PROJECT_NAME}_exe)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/gtest_main.cpp test/test_hello_world.cpp)
  set(TEST_HELLO_WORLD_EXE test_hello_world)
  ament_add_gtest(${TEST_HELLO_WORLD_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_HELLO_WORLD_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
