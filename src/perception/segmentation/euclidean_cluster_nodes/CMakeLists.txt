# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Build the nodes
project(euclidean_cluster_nodes)

## dependencies
find_package(autoware_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(euclidean_cluster REQUIRED)
find_package(geometry REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(voxel_grid_nodes REQUIRED)

# Library
set(CLUSTER_NODE_LIB euclidean_cluster_node)
add_library(${CLUSTER_NODE_LIB} SHARED
  # include/euclidean_cluster_nodes/details/common.hpp
  include/euclidean_cluster_nodes/euclidean_cluster_node.hpp
  # src/common.cpp
  src/euclidean_cluster_node.cpp)
target_include_directories(${CLUSTER_NODE_LIB} PUBLIC "include")
autoware_set_compile_options(${CLUSTER_NODE_LIB})
ament_target_dependencies(${CLUSTER_NODE_LIB}
  "rclcpp"
  "autoware_auto_msgs"
  "euclidean_cluster"
  "geometry"
  "sensor_msgs"
  "voxel_grid_nodes")

# Executable
set(CLUSTER_MAIN euclidean_cluster_exe)
add_executable(${CLUSTER_MAIN} src/euclidean_cluster_main.cpp)
autoware_set_compile_options(${CLUSTER_MAIN})
target_link_libraries(${CLUSTER_MAIN} ${CLUSTER_NODE_LIB})

## Testing
if(BUILD_TESTING)
  # Static checking only if built via ament
  autoware_static_code_analysis()
  find_package(ament_cmake_pclint REQUIRED)
  ament_pclint(CHECK_HEADERS)
endif()
# Ament Exporting
autoware_install(LIBRARIES ${CLUSTER_NODE_LIB}
  EXECUTABLES ${CLUSTER_MAIN}
  HAS_PARAM
  HAS_INCLUDE)
ament_export_dependencies(
  "rclcpp"
  "autoware_auto_msgs"
  "euclidean_cluster"
  "geometry"
  "sensor_msgs"
  "voxel_grid_nodes")
ament_package()
