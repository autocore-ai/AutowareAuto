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

project(detection_2d_visualizer)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(GROUND_TRUTH_VISUALIZER_NODE_SRC
  src/ground_truth_visualizer_node.cpp
  src/detection_2d_visualizer_node.cpp
  src/utils.cpp)

set(GROUND_TRUTH_VISUALIZER_NODE_HEADERS
    include/detection_2d_visualizer/ground_truth_visualizer_node.hpp
    include/detection_2d_visualizer/detection_2d_visualizer_node.hpp
    include/detection_2d_visualizer/utils.hpp)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${GROUND_TRUTH_VISUALIZER_NODE_SRC}
  ${GROUND_TRUTH_VISUALIZER_NODE_HEADERS}
)

autoware_set_compile_options(${PROJECT_NAME}_node)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::detection_2d_visualizer::GroundTruthVisualizerNode"
  EXECUTABLE ground_truth_visualizer_node_exe
)

rclcpp_components_register_node(${PROJECT_NAME}_node
    PLUGIN "autoware::detection_2d_visualizer::Detection2dVisualizerNode"
    EXECUTABLE detection_2d_visualizer_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package generation and installing
ament_auto_package()