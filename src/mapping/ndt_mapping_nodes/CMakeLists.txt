# Copyright 2020 the Autoware Foundation
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
project(ndt_mapping_nodes)

#dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(
${PROJECT_NAME} SHARED
        include/ndt_mapping_nodes/visibility_control.hpp
        include/ndt_mapping_nodes/ndt_mapping_nodes.hpp
        src/ndt_mapping_nodes.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

set(MAPPER_NODE_EXE ndt_mapper_node_exe)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::mapping::ndt_mapping_nodes::P2DNDTVoxelMapperNode<>"
  EXECUTABLE ${MAPPER_NODE_EXE}
)

# TODO(yunus.caliskan): Remove once #978 is fixed.
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-Og")
else()
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-O3 -ftree-vectorize")
endif()

## Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${MAPPER_NODE_EXE})

  find_package(ament_cmake_gtest REQUIRED)
  # TODO(yunus.caliskan): figure out why we need this when it gets exported by point_cloud_mapping
  find_package(PCL 1.8 REQUIRED COMPONENTS io)

  ament_add_gtest(test_${PROJECT_NAME} test/test_ndt_mapping_nodes.cpp)
  autoware_set_compile_options(test_${PROJECT_NAME})
  target_compile_options(test_${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-double-promotion -Wno-useless-cast)

  add_dependencies(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${PCL_LIBRARIES})
endif()
# TODO(yunus.caliskan): Remove after #1098
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-float-conversion -Wno-conversion
-Wno-double-promotion)
ament_auto_package(INSTALL_TO_SHARE param launch)
