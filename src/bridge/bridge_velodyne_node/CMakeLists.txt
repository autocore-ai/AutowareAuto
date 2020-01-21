# Copyright 2019 Tier IV, Inc.
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

project(bridge_velodyne_node)

# dependencies
find_package(autoware_auto_cmake REQUIRED)  # Required on all packages!
find_package(ament_cmake REQUIRED)

# Testing
if(BUILD_TESTING)
  # TODO: Integration testing cannot work on Gitlab CI.
  # After solving the issue, this comment-out will be removed.
  # find_package(launch_testing_ament_cmake)
  # add_launch_test(test/bridge_velodyne_node_test.py
  #  TIMEOUT 200)
endif()

# Install stuff
autoware_install(
  HAS_LAUNCH
)

# TODO: Integration testing cannot work on Gitlab CI.
# Install testing stuff
# install(
#   DIRECTORY test/
#   DESTINATION share/${PROJECT_NAME}/test/
# )
# Ament Exporting
ament_export_dependencies("rclcpp")
ament_package()
