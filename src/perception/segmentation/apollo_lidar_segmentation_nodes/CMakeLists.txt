# Copyright 2021 Arm Ltd.
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

cmake_minimum_required(VERSION 3.6)
project(apollo_lidar_segmentation_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Check that a neural network was available for apollo_lidar_segmentation to be built.
if(apollo_lidar_segmentation_FOUND)
  # Set apollo_lidar_segmentation includes as "SYSTEM" to ignore compiler errors on PCL headers
  include_directories(SYSTEM "${apollo_lidar_segmentation_INCLUDE_DIRS}")

  # Library
  ament_auto_add_library(${PROJECT_NAME} SHARED
    include/apollo_lidar_segmentation_nodes/apollo_lidar_segmentation_node.hpp
    src/apollo_lidar_segmentation_node.cpp
  )
  autoware_set_compile_options(${PROJECT_NAME})
  target_link_libraries(${PROJECT_NAME} ${apollo_lidar_segmentation_LIBRARIES})

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "autoware::perception::segmentation::apollo_lidar_segmentation_nodes::ApolloLidarSegmentationNode"
    EXECUTABLE ${PROJECT_NAME}_exe
  )

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    add_ros_test(
      test/launch.test.py
      TIMEOUT "30"
    )
  endif()

  ament_auto_package(INSTALL_TO_SHARE
    launch
    param
  )
else()
  message(WARNING "apollo_lidar_segmentation not found, skipping package.")
endif()
