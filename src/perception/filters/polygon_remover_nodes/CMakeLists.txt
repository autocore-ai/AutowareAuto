# Copyright 2021 The Autoware Foundation
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

project(polygon_remover_nodes)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(NODE_NAME polygon_remover_node)
ament_auto_add_library(${NODE_NAME} SHARED
  include/polygon_remover_nodes/visibility_control.hpp
  include/polygon_remover_nodes/polygon_remover_node.hpp
  src/polygon_remover_node.cpp)
autoware_set_compile_options(${NODE_NAME})
rclcpp_components_register_node(${NODE_NAME}
  PLUGIN "autoware::perception::filters::polygon_remover_nodes::PolygonRemoverNode"
  EXECUTABLE ${NODE_NAME}_exe)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_polygon_remover_nodes.cpp)
  set(TEST_POLYGON_REMOVER_NODES_EXE test_polygon_remover_nodes)
  ament_add_gtest(${TEST_POLYGON_REMOVER_NODES_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_POLYGON_REMOVER_NODES_EXE})
  target_link_libraries(${TEST_POLYGON_REMOVER_NODES_EXE} ${NODE_NAME})

endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param)
