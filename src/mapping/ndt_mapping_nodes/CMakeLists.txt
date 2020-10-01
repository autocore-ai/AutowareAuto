# Copyright 2020 Apex.AI, Inc.
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
  PLUGIN "autoware::mapping::ndt_mapping_nodes::P2DNDTVoxelMapperNode"
  EXECUTABLE ${MAPPER_NODE_EXE}
)

# turn off warnings to be able to successfully compile upstream ros packages
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
set(ROS_NO_WARN_LIST
        -Wno-sign-conversion
        -Wno-conversion
        -Wno-old-style-cast
        -Wno-useless-cast
        -Wno-double-promotion
        -Wno-nonnull-compare
        -Wuseless-cast)
target_compile_options(${PROJECT_NAME} PRIVATE ${ROS_NO_WARN_LIST})

## Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME} test/test_ndt_mapping_nodes.cpp)
  add_dependencies(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
endif()

ament_auto_package(INSTALL_TO_SHARE param launch)
