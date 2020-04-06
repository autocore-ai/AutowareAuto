# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(lidar_utils)

find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
find_package(autoware_auto_geometry REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# includes
include_directories(include)
# Disable warnings due to external dependencies (Eigen)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} SHARED
  include/lidar_utils/point_cloud_utils.hpp
  include/lidar_utils/lidar_utils.hpp
  src/point_cloud_utils.cpp)

autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  "autoware_auto_common"
  "autoware_auto_geometry"
  "geometry_msgs"
  "sensor_msgs"
  "rclcpp"
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
    ament_target_dependencies(test_fast_atan2
      "autoware_auto_common")
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
  "autoware_auto_geometry"
  "rclcpp"
  "sensor_msgs"
)
ament_package(
  CONFIG_EXTRAS_POST "lidar_utils-extras.cmake"
)
