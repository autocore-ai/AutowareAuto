# Copyright 2021 The Autoware Foundation
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

project(vehicle_constants_manager_nodes)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(NAME_NODE vehicle_constants_manager_node)
ament_auto_add_library(${NAME_NODE} SHARED
  include/vehicle_constants_manager_nodes/visibility_control.hpp
  include/vehicle_constants_manager_nodes/vehicle_constants_manager_node.hpp
  src/vehicle_constants_manager_node.cpp)
autoware_set_compile_options(${NAME_NODE})
rclcpp_components_register_node(${NAME_NODE}
  PLUGIN "autoware::common::vehicle_constants_manager_node::VehicleConstantsManagerNode"
  EXECUTABLE ${NAME_NODE}_exe)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_vehicle_constants_manager_node.cpp)
  set(TEST_VEHICLE_CONSTANTS_MANAGER_NODE_EXE test_vehicle_constants_manager_node)
  ament_add_gtest(${TEST_VEHICLE_CONSTANTS_MANAGER_NODE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_VEHICLE_CONSTANTS_MANAGER_NODE_EXE})
  target_link_libraries(${TEST_VEHICLE_CONSTANTS_MANAGER_NODE_EXE} ${NAME_NODE})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param)
