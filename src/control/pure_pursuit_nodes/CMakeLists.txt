# Copyright 2019 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(pure_pursuit_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/pure_pursuit_nodes/pure_pursuit_node.hpp
  include/pure_pursuit_nodes/visibility_control.hpp
  src/pure_pursuit_node.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

set(PURE_PURSUIT_NODE_EXE pure_pursuit_node_exe)
ament_auto_add_executable(${PURE_PURSUIT_NODE_EXE} src/pure_pursuit_node_main.cpp)
autoware_set_compile_options(${PURE_PURSUIT_NODE_EXE})

if(BUILD_TESTING)
  # run common linters
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  set(PURE_PURSUIT_NODE_TEST pure_pursuit_integration_test_exe)
  ament_auto_add_executable(${PURE_PURSUIT_NODE_TEST} test/src/pure_pursuit_integration_test.cpp)
  autoware_set_compile_options(${PURE_PURSUIT_NODE_TEST})

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${PURE_PURSUIT_NODE_EXE})
endif()

## install stuff
ament_auto_package(
  INSTALL_TO_SHARE param
)
