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
project(spinnaker_camera_nodes)

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
  ament_auto_add_library(spinnaker_camera_node SHARED
    "include/spinnaker_camera_nodes/spinnaker_camera_node.hpp"
    "src/spinnaker_camera_node.cpp")
  autoware_set_compile_options(spinnaker_camera_node)

  target_compile_options(spinnaker_camera_node PRIVATE -Wno-extra
    -Wno-ignored-qualifiers -Wno-old-style-cast -Wno-overloaded-virtual)

  rclcpp_components_register_node(spinnaker_camera_node
    PLUGIN "autoware::drivers::camera::SpinnakerCameraNode"
    EXECUTABLE spinnaker_camera_node_exe
  )

  ## Testing
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    add_ros_test(
      test/spinnaker_camera_node_bad.test.py
      TIMEOUT "10"
    )
  endif()
else()
  message(WARNING "SPINNAKER SDK not found, so spinnaker_camera_nodes could not be built.")
endif()

ament_auto_package(
  INSTALL_TO_SHARE param
)
