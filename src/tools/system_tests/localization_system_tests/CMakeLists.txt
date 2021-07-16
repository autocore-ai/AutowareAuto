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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)

project(localization_system_tests)

# dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
    include/localization_system_tests/localization_evaluation_node.hpp
    src/localization_evaluation_node.cpp)
autoware_set_compile_options(${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "localization_system_tests::LocalizationEvaluationNode"
    EXECUTABLE ${PROJECT_NAME}_exe
    )

ament_auto_package(
    INSTALL_TO_SHARE
    param
    launch
    config/rviz2
)
