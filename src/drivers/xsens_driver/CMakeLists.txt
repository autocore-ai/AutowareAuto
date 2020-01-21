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
project(xsens_driver)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

ament_auto_find_build_dependencies(REQUIRED ${requirement_dependencies})

include_directories(include)

### Build driver library
set(required_dependencies
  "autoware_auto_common"
  "lidar_utils"
  "sensor_msgs"
)

ament_auto_add_library(${PROJECT_NAME}
        "include/xsens_driver/xsens_common.hpp"
        "include/xsens_driver/xsens_base_translator.hpp"
        "include/xsens_driver/xsens_imu_translator.hpp"
        "include/xsens_driver/visibility_control.hpp"
        "src/xsens_common.cpp"
        "src/xsens_gps_config.cpp"
        "src/xsens_gps_translator.cpp"
        "src/xsens_imu_config.cpp"
        "src/xsens_imu_translator.cpp")
target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(${PROJECT_NAME}
  ${required_dependencies})
autoware_set_compile_options(${PROJECT_NAME})

## Testing
if(BUILD_TESTING)
    # Static checking only if built via ament
    autoware_static_code_analysis()
    include_directories(include)
    # gtest
    find_package(ament_cmake_gtest REQUIRED)
    set(XSENS_IMU_GTEST xsens_imu_gtest)
    ament_add_gtest(${XSENS_IMU_GTEST}
      "test/src/test_xsens_imu.cpp")
    target_include_directories(${XSENS_IMU_GTEST} PRIVATE test/include include)
    target_link_libraries(${XSENS_IMU_GTEST} ${PROJECT_NAME})

    set(XSENS_GPS_GTEST xsens_gps_gtest)
    ament_add_gtest(${XSENS_GPS_GTEST}
      "test/src/test_xsens_gps.cpp")
    target_include_directories(${XSENS_GPS_GTEST} PRIVATE test/include include)
    target_link_libraries(${XSENS_GPS_GTEST} ${PROJECT_NAME})
endif()

autoware_install(LIBRARIES ${PROJECT_NAME} HAS_INCLUDE)
ament_auto_package()
