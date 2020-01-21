# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(lidar_utils)

find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# includes
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  include/lidar_utils/point_cloud_utils.hpp
  include/lidar_utils/lidar_utils.hpp
  src/point_cloud_utils.cpp)

autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  "autoware_auto_common"
  "sensor_msgs"
  "rclcpp"
  "rclcpp_lifecycle"
)

if(BUILD_TESTING)
    # run linters
    autoware_static_code_analysis()

    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_fast_atan2
      test/src/test_fast_atan2.cpp
    )
    target_include_directories(test_fast_atan2
      PRIVATE "include"
    )
    ament_target_dependencies(test_fast_atan2 "autoware_auto_common")
endif()

autoware_install(
  HAS_INCLUDE
  LIBRARIES ${PROJECT_NAME}
)

# workaround to disable sign conversion errors from sensor_msgs::PointCloud2Iterator
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion)

# Ament Exporting
ament_export_dependencies(
  "autoware_auto_common"
  "rclcpp"
  "sensor_msgs"
  "rclcpp_lifecycle")
ament_package()
