# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(localization_nodes)

#dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(localization_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

# includes
include_directories(include)

set(LOCALIZATION_NODES_LIB_SRC
    src/localization_node.cpp
)

set(LOCALIZATION_NODES_LIB_HEADERS
    include/localization_nodes/visibility_control.hpp
    include/localization_nodes/localization_node.hpp)

add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_NODES_LIB_SRC}
        ${LOCALIZATION_NODES_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} "localization_common" "rclcpp" "tf2")

autoware_install(HAS_INCLUDE)
ament_package()
