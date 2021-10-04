# Copyright 2020 The Autoware Foundation
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

project(lanelet2_map_provider)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(GEOGRAPHICLIB REQUIRED)

set(LANELET2_MAP_PROVIDER_LIB_SRC
  src/lanelet2_map_provider.cpp
)

set(LANELET2_MAP_PROVIDER_LIB_HEADERS
  include/lanelet2_map_provider/lanelet2_map_provider.hpp
  include/lanelet2_map_provider/visibility_control.hpp
)

set(LANELET2_MAP_PROVIDER_NODE_SRC
  src/lanelet2_map_provider_node.cpp
)

set(LANELET2_MAP_PROVIDER_NODE_HEADERS
  include/lanelet2_map_provider/lanelet2_map_provider_node.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${LANELET2_MAP_PROVIDER_LIB_SRC}
  ${LANELET2_MAP_PROVIDER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  ${GeographicLib_LIBRARIES})


set(LANELET2_MAP_NODES lanelet2_map_nodes)

set(LANELET2_MAP_VISUALIZER_SRC
  src/lanelet2_map_visualizer.cpp
)
set(LANELET2_MAP_VISUALIZER_HEADERS
  include/lanelet2_map_provider/lanelet2_map_visualizer.hpp
  include/lanelet2_map_provider/visibility_control.hpp
)
ament_auto_add_library(${LANELET2_MAP_NODES} SHARED
  ${LANELET2_MAP_PROVIDER_NODE_SRC}
  ${LANELET2_MAP_PROVIDER_NODE_HEADERS}
  ${LANELET2_MAP_VISUALIZER_SRC}
  ${LANELET2_MAP_VISUALIZER_HEADERS}
)
autoware_set_compile_options(${LANELET2_MAP_NODES})

target_link_libraries(${LANELET2_MAP_NODES}
  ${GeographicLib_LIBRARIES})

set(PROVIDER_NODE_NAME ${PROJECT_NAME}_exe)
rclcpp_components_register_node(${LANELET2_MAP_NODES}
  PLUGIN "autoware::lanelet2_map_provider::Lanelet2MapProviderNode"
  EXECUTABLE ${PROVIDER_NODE_NAME}
)

set(VISUALIZER_NODE_NAME lanelet2_map_visualizer_exe)
rclcpp_components_register_node(${LANELET2_MAP_NODES}
  PLUGIN "autoware::lanelet2_map_provider::Lanelet2MapVisualizer"
  EXECUTABLE ${VISUALIZER_NODE_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/lanelet2_map_provider_launch.test.py
    TIMEOUT "30"
  )

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${VISUALIZER_NODE_NAME})

  # Unit tests
  set(TEST_SOURCES test/test_lanelet2_map_provider.cpp)
  set(TEST_LANELET2_MAP_PROVIDER_EXE test_lanelet2_map_provider)
  ament_add_gtest(${TEST_LANELET2_MAP_PROVIDER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_LANELET2_MAP_PROVIDER_EXE})
  target_link_libraries(${TEST_LANELET2_MAP_PROVIDER_EXE}
    ${PROJECT_NAME}
  )
  add_dependencies(${TEST_LANELET2_MAP_PROVIDER_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  data
  launch
  param
)
