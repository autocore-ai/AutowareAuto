# Copyright 2020-2021 Apex.AI, Inc., Arm Limited
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
project(covariance_insertion_nodes)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(MESSAGE_MODIFYING_NODE_LIB_SRC
  src/covariance_insertion_node.cpp
  src/convert.cpp
)

set(MESSAGE_MODIFYING_NODE_LIB_HEADERS
  include/covariance_insertion_nodes/covariance_insertion_node.hpp
  include/covariance_insertion_nodes/convert.hpp
  include/covariance_insertion_nodes/visibility_control.hpp
)

# generate library
ament_auto_add_library(covariance_insertion_node SHARED
  ${MESSAGE_MODIFYING_NODE_LIB_SRC}
  ${MESSAGE_MODIFYING_NODE_LIB_HEADERS}
)
autoware_set_compile_options(covariance_insertion_node)

rclcpp_components_register_node(covariance_insertion_node
  PLUGIN "autoware::covariance_insertion_nodes::CovarianceInsertionNode"
  EXECUTABLE covariance_insertion_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_covariance_insertion_node.cpp)
  set(TEST_MESSAGE_MODIFYING_NODE_EXE test_covariance_insertion_node)
  ament_add_gtest(${TEST_MESSAGE_MODIFYING_NODE_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_MESSAGE_MODIFYING_NODE_EXE} covariance_insertion_node)
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE param)
