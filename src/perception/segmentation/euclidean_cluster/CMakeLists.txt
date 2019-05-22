
# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(euclidean_cluster)

## dependencies
find_package(autoware_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry REQUIRED)
find_package(lidar_utils REQUIRED)
find_package(autoware_auto_msgs REQUIRED)

# includes
add_library(${PROJECT_NAME} SHARED
  include/euclidean_cluster/euclidean_cluster.hpp
  include/euclidean_cluster/visibility_control.hpp
  src/euclidean_cluster.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC "include")
autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  "geometry"
  "lidar_utils"
  "autoware_auto_msgs")
target_compile_options(${PROJECT_NAME} PUBLIC "-O0")

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
  find_package(ament_cmake_pclint REQUIRED)
  ament_pclint(CHECK_HEADERS)

  # gtest
  set(EUCLIDEAN_CLUSTER_GTEST euclidean_cluster_gtest)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${EUCLIDEAN_CLUSTER_GTEST} test/src/test_euclidean_cluster.cpp)
  target_include_directories(${EUCLIDEAN_CLUSTER_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${EUCLIDEAN_CLUSTER_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${EUCLIDEAN_CLUSTER_GTEST} "geometry")
endif()

autoware_install(LIBRARIES ${PROJECT_NAME} HAS_INCLUDE)
# Ament Exporting
ament_export_dependencies(
  "geometry"
  "lidar_utils"
  "autoware_auto_msgs")
ament_package()
