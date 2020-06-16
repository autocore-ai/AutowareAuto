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

cmake_minimum_required(VERSION 3.5)

### Build the nodes
project(spinnaker_camera_node)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

## dependencies
find_package(ament_cmake_auto REQUIRED)

# Even though Spinnaker is not required to build this package,
# we must rely on it to only build this package if it exists.
# The dependent package which does rely on Spinnaker (spinnaker_camera_driver)
# must exist in the ament index to avoid an error on sourcing
# the install space so we can't use that as the deciding factor.
find_package(SPINNAKER)

if(SPINNAKER_FOUND)
  ament_auto_find_build_dependencies()
  ament_auto_add_library(${PROJECT_NAME} SHARED
    "include/spinnaker_camera_node/spinnaker_camera_node.hpp"
    "src/spinnaker_camera_node.cpp")
  autoware_set_compile_options(${PROJECT_NAME})

  set(CAMERA_EXECUTABLE "${PROJECT_NAME}_exe")
  ament_auto_add_executable(${CAMERA_EXECUTABLE}
    "src/spinnaker_camera_node_main.cpp")
  autoware_set_compile_options(${CAMERA_EXECUTABLE})

  ## Testing
  if(BUILD_TESTING)
    # Static checking only if built via ament
    autoware_static_code_analysis()
    include_directories(include)

    find_package(ros_testing REQUIRED)
    add_ros_test(
      test/spinnaker_camera_node_bad.test.py
      TIMEOUT "10"
    )
  endif()
else()
  message(WARNING "SPINNAKER SDK not found, so spinnaker_camera_node could not be built.")
endif()

ament_auto_package(
  INSTALL_TO_SHARE param
)
