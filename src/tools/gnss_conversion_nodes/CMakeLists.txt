# Copyright 2021 Apex.AI, Inc.
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
project(gnss_conversion_nodes)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(GEOGRAPHICLIB REQUIRED)
find_package(Eigen3 REQUIRED)


ament_auto_find_build_dependencies()

set(GNSS_CONVERSION_NODE_LIB_HEADERS
  include/gnss_conversion_nodes/gnss_conversion_node.hpp
  include/gnss_conversion_nodes/visibility_control.hpp
)

set(GNSS_CONVERSION_NODE_LIB_SRC
  src/gnss_conversion_node.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${GNSS_CONVERSION_NODE_LIB_SRC}
  ${GNSS_CONVERSION_NODE_LIB_HEADERS}
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${GeographicLib_LIBRARIES})
autoware_set_compile_options(${PROJECT_NAME})
# Allow old-style-cast until the issue in THROTTLE logging is fixed.
# See: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/955
target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-error=old-style-cast")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::${PROJECT_NAME}::GnssConversionNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)


# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_gnss_conversion_node.cpp)
  set(TEST_GNSS_CONVERTION_NODE test_gnss_conversion_node)
  ament_add_gtest(${TEST_GNSS_CONVERTION_NODE} ${TEST_SOURCES})
  target_link_libraries(${TEST_GNSS_CONVERTION_NODE} ${PROJECT_NAME})
  ament_target_dependencies(${TEST_GNSS_CONVERTION_NODE}
      "rclcpp"
      "rclcpp_components"
      "autoware_auto_common"
      "autoware_auto_msgs"
      "sensor_msgs"
  )
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE param)
