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

project(ne_raptor_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(NE_RAPTOR_INTERFACE_LIB_SRC
  src/ne_raptor_interface.cpp
)

set(NE_RAPTOR_INTERFACE_LIB_HEADERS
  include/ne_raptor_interface/ne_raptor_interface.hpp
  include/ne_raptor_interface/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${NE_RAPTOR_INTERFACE_LIB_SRC}
  ${NE_RAPTOR_INTERFACE_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(NE_RAPTOR_INTERFACE_NODE_SRC
  src/ne_raptor_interface_node.cpp
)

set(NE_RAPTOR_INTERFACE_NODE_HEADERS
  include/ne_raptor_interface/ne_raptor_interface_node.hpp
)

# generate component node library
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${NE_RAPTOR_INTERFACE_NODE_SRC}
  ${NE_RAPTOR_INTERFACE_NODE_HEADERS}
)

# TODO(NE_Raptor) : RCLCPP_ERROR_THROTTLE() has built-in old-style casts.
# TODO(NE_Raptor) : Temporary workaround until this is fixed.
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-old-style-cast)

autoware_set_compile_options(${PROJECT_NAME}_node)

set(NODE_NAME ${PROJECT_NAME}_node_exe)
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::ne_raptor_interface::NERaptorInterfaceNode"
  EXECUTABLE ${NODE_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})

  # Unit tests
  set(TEST_SOURCES
    test/test_ne_raptor_interface.cpp
    test/test_ne_raptor_interface_listener.cpp
    test/test_ne_raptor_interface_talker.cpp
  )
  set(TEST_NE_RAPTOR_INTERFACE_EXE test_ne_raptor_interface)
  ament_add_gtest(${TEST_NE_RAPTOR_INTERFACE_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_NE_RAPTOR_INTERFACE_EXE})
  target_link_libraries(${TEST_NE_RAPTOR_INTERFACE_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  param
)
