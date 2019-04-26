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

## dependencies
find_package(autoware_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lidar_utils REQUIRED)

include_directories(include)


### Build driver library
add_library(${PROJECT_NAME} SHARED
        "include/velodyne_driver/vlp16_translator.hpp"
        "include/velodyne_driver/visibility_control.hpp"
        "src/vlp16_config.cpp"
        "src/vlp16_translator.cpp")
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "lidar_utils"
)
autoware_set_compile_options(${PROJECT_NAME})

## Testing
if(BUILD_TESTING)
    # Static checking only if built via ament
    autoware_static_code_analysis()
    include_directories(include)
    # gtest
    find_package(ament_cmake_gtest REQUIRED)
    set(VELODYNE_GTEST velodyne_gtest)
    ament_add_gtest(${VELODYNE_GTEST}
            "test/include/test_driver.h"
            "test/src/test_velodyne.cpp")
    target_include_directories(${VELODYNE_GTEST} PRIVATE test/include include)
    target_link_libraries(${VELODYNE_GTEST} ${PROJECT_NAME})
endif()

autoware_install(LIBRARIES ${PROJECT_NAME} HAS_INCLUDE)
ament_package()
