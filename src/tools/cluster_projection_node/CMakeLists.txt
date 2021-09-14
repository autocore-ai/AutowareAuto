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

project(cluster_projection_node)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CLUSTER_PROJECTION_NODE_SRC
    src/cluster_projection_node.cpp
)

set(CLUSTER_PROJECTION_NODE_HEADERS
    include/cluster_projection_node/cluster_projection_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${CLUSTER_PROJECTION_NODE_SRC}
  ${CLUSTER_PROJECTION_NODE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::cluster_projection_node::ClusterProjectionNode"
  EXECUTABLE cluster_projection_node_exe
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE param)