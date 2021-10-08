# Copyright 2021 the Autoware Foundation
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

project(prediction_nodes)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(PREDICTION_NODES_LIB_SRC
  src/prediction_node.cpp
)

set(PREDICTION_NODES_LIB_HEADERS
  include/prediction_nodes/prediction_node.hpp
  include/prediction_nodes/visibility_control.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node_lib SHARED
  ${PREDICTION_NODES_LIB_HEADERS}
  ${PREDICTION_NODES_LIB_SRC}
)

autoware_set_compile_options(${PROJECT_NAME}_node_lib)
rclcpp_components_register_node(${PROJECT_NAME}_node_lib
  PLUGIN "autoware::prediction::PredictionNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(fake_test_node REQUIRED)

  # Unit tests
  set(TEST_SOURCES
    test/test_prediction_nodes.cpp
  )
  set(TEST_PREDICTION_NODES_EXE test_prediction_nodes)
  ament_add_gtest(${TEST_PREDICTION_NODES_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_PREDICTION_NODES_EXE})
  target_link_libraries(${TEST_PREDICTION_NODES_EXE} ${PROJECT_NAME}_node_lib)
  add_dependencies(${TEST_PREDICTION_NODES_EXE} ${PROJECT_NAME}_node_lib)
  ament_target_dependencies(${TEST_PREDICTION_NODES_EXE}
    "fake_test_node"
  )

  find_package(ros_testing REQUIRED)
  add_ros_test(
    test/prediction_nodes_launch.test.py
    TIMEOUT "30"
  )
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
