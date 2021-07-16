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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

cmake_minimum_required(VERSION 3.5)

project(ground_truth_detections)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(GROUND_TRUTH_DETECTIONS_LIB_SRC
  src/ground_truth_detections.cpp
)

set(GROUND_TRUTH_DETECTIONS_LIB_HEADERS
  include/ground_truth_detections/ground_truth_detections.hpp
  include/ground_truth_detections/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${GROUND_TRUTH_DETECTIONS_LIB_SRC}
  ${GROUND_TRUTH_DETECTIONS_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(GROUND_TRUTH_DETECTIONS_NODE_SRC
  src/ground_truth_detections_node.cpp
)

set(GROUND_TRUTH_DETECTIONS_NODE_HEADERS
  include/ground_truth_detections/ground_truth_detections_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${GROUND_TRUTH_DETECTIONS_NODE_SRC}
  ${GROUND_TRUTH_DETECTIONS_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME}_node)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::ground_truth_detections::GroundTruthDetectionsNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(fake_test_node REQUIRED)

  # Component test
  set(TEST_SOURCES test/test_ground_truth_detections_node.cpp)
  set(TEST_GROUND_TRUTH_DETECTIONS_EXE test_ground_truth_detections)
  ament_add_gtest(${TEST_GROUND_TRUTH_DETECTIONS_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_GROUND_TRUTH_DETECTIONS_EXE})
  target_link_libraries(${TEST_GROUND_TRUTH_DETECTIONS_EXE} ${PROJECT_NAME} ${PROJECT_NAME}_node)
  add_dependencies(${TEST_GROUND_TRUTH_DETECTIONS_EXE} ${PROJECT_NAME}_node)
  ament_target_dependencies(${TEST_GROUND_TRUTH_DETECTIONS_EXE}
    "fake_test_node"
  )
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  config
)
