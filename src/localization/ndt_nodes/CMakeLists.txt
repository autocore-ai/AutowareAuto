# Copyright 2019-2020 Apex.AI, Inc., Arm Limited
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(ndt_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)

find_package(PCL 1.8 REQUIRED COMPONENTS io)

ament_auto_find_build_dependencies()
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

set(NDT_MAP_PUBLISHER_NODE_LIB_SRC
  src/map_publisher.cpp
)

set(NDT_MAP_PUBLISHER_NODE_LIB_HEADERS
  include/ndt_nodes/map_publisher.hpp
)

set(NDT_MAP_PUBLISHER_NODE_LIB ndt_map_publisher)
ament_auto_add_library(${NDT_MAP_PUBLISHER_NODE_LIB} SHARED
  ${NDT_MAP_PUBLISHER_NODE_LIB_SRC}
  ${NDT_MAP_PUBLISHER_NODE_LIB_HEADERS}
)
autoware_set_compile_options(${NDT_MAP_PUBLISHER_NODE_LIB})

target_link_libraries(${NDT_MAP_PUBLISHER_NODE_LIB}
  ${PCL_LIBRARIES})

rclcpp_components_register_node(${NDT_MAP_PUBLISHER_NODE_LIB}
  PLUGIN "autoware::localization::ndt_nodes::NDTMapPublisherNode"
  EXECUTABLE ${NDT_MAP_PUBLISHER_NODE_LIB}_exe
)

set(P2D_NDT_LOCALIZER_NODE_LIB_SRC
  src/p2d_ndt_localizer.cpp
)

set(P2D_NDT_LOCALIZER_NODE_LIB_HEADERS
  include/ndt_nodes/ndt_localizer_nodes.hpp
)

set(P2D_NDT_LOCALIZER_NODE_LIB p2d_ndt_localizer)
ament_auto_add_library(${P2D_NDT_LOCALIZER_NODE_LIB} SHARED
  ${P2D_NDT_LOCALIZER_NODE_LIB_SRC}
  ${P2D_NDT_LOCALIZER_NODE_LIB_HEADERS}
)
autoware_set_compile_options(${P2D_NDT_LOCALIZER_NODE_LIB})

target_link_libraries(${P2D_NDT_LOCALIZER_NODE_LIB}
  ${PCL_LIBRARIES})

rclcpp_components_register_node(${P2D_NDT_LOCALIZER_NODE_LIB}
  PLUGIN "autoware::localization::ndt_nodes::P2DNDTLocalizerNodeComponent"
  EXECUTABLE ${P2D_NDT_LOCALIZER_NODE_LIB}_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(NDT_NODES_TEST ndt_nodes_gtest)

  ament_add_gtest(${NDT_NODES_TEST}
    test/test_map_publisher.cpp
    WORKING_DIRECTORY /tmp)
  target_link_libraries(${NDT_NODES_TEST}
    ${NDT_MAP_PUBLISHER_NODE_LIB}
    ${YAML_CPP_LIBRARIES})
endif()

# required for tf2
target_include_directories(${NDT_MAP_PUBLISHER_NODE_LIB} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
target_compile_options(${NDT_MAP_PUBLISHER_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

target_include_directories(${P2D_NDT_LOCALIZER_NODE_LIB} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
target_compile_options(${P2D_NDT_LOCALIZER_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)