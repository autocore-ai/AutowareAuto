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

project(outlier_filter_nodes)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

set(OUTLIER_FILTER_NODES_SRC
  src/radius_search_2d_filter_node.cpp
  src/voxel_grid_outlier_filter_node.cpp
)

set(OUTLIER_FILTER_NODES_HEADERS
  include/outlier_filter_nodes/radius_search_2d_filter_node.hpp
  include/outlier_filter_nodes/voxel_grid_outlier_filter_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OUTLIER_FILTER_NODES_SRC}
  ${OUTLIER_FILTER_NODES_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::perception::filters::outlier_filter_nodes::RadiusSearch2DFilterNode"
  EXECUTABLE radius_search_2d_filter_node_exe
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::perception::filters::outlier_filter_nodes::VoxelGridOutlierFilterNode"
  EXECUTABLE voxel_grid_outlier_filter_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/radius_search_2d_filter_node_launch.test.py
    TIMEOUT "30"
  )

  add_ros_test(
    test/voxel_grid_outlier_filter_node_launch.test.py
    TIMEOUT "30"
  )
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)
