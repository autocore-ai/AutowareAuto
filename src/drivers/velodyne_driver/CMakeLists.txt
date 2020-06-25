# Copyright 2018 Apex.AI, Inc.
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

### Build the nodes
project(velodyne_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build driver library
ament_auto_add_library(${PROJECT_NAME} SHARED
        "include/velodyne_driver/velodyne_translator.hpp"
        "include/velodyne_driver/vlp16_data.hpp"
        "include/velodyne_driver/vls128_data.hpp"
        "include/velodyne_driver/common.hpp"
        "src/vlp16_data.cpp"
        "src/vlS128_data.cpp"
        "src/velodyne_translator.cpp")

autoware_set_compile_options(${PROJECT_NAME})

## Testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # gtest
    set(VELODYNE_GTEST velodyne_gtest)
    ament_add_gtest(${VELODYNE_GTEST}
            "test/src/test_velodyne.cpp")
    target_include_directories(${VELODYNE_GTEST} PRIVATE test/include include)
    target_link_libraries(${VELODYNE_GTEST} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
