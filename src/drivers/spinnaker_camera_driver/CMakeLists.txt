# Copyright 2020 Apex.AI, Inc.
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

### Build the nodes
project(spinnaker_camera_driver)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(SPINNAKER)

if(SPINNAKER_FOUND)
  ament_auto_find_build_dependencies()
  ament_auto_add_library(${PROJECT_NAME} SHARED
    "include/spinnaker_camera_driver/visibility_control.hpp"
    "include/spinnaker_camera_driver/system_wrapper.hpp"
    "include/spinnaker_camera_driver/camera_settings.hpp"
    "include/spinnaker_camera_driver/camera_wrapper.hpp"
    "include/spinnaker_camera_driver/camera_list_wrapper.hpp"
    "src/system_wrapper.cpp"
    "src/camera_settings.cpp"
    "src/camera_wrapper.cpp"
    "src/camera_list_wrapper.cpp")
  autoware_set_compile_options(${PROJECT_NAME})
  ## SPINNAKER is not handled by ament, so we need the manual steps below.
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${SPINNAKER_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${SPINNAKER_LIBRARIES})
  # These are external libraries and include folders that we want to be visible to the packages
  # that depend on this one.
  ament_export_include_directories(${SPINNAKER_INCLUDE_DIRS})
  ament_export_libraries(${SPINNAKER_LIBRARIES})

  ## Testing
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
else()
  message(WARNING "SPINNAKER SDK not found, so ${PROJECT_NAME} could not be built.")
endif()

ament_auto_package()
