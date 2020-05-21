# Copyright 2019 Apex.AI, Inc.
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

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(GEOGRAPHICLIB REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(ament_cmake_auto REQUIRED)

find_package(PCL 1.8 REQUIRED COMPONENTS io)

ament_auto_find_build_dependencies()
include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

set(NDT_NODES_LIB_SRC
    src/map_publisher.cpp
)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${NDT_NODES_LIB_SRC}
)

autoware_set_compile_options(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

set(MAP_PUBLISHER_EXE ndt_map_publisher_exe)
ament_auto_add_executable(${MAP_PUBLISHER_EXE} src/map_publisher_main.cpp)
autoware_set_compile_options(${MAP_PUBLISHER_EXE})

set(P2D_NDT_NODE p2d_ndt_localizer_exe)
ament_auto_add_executable(${P2D_NDT_NODE} src/p2d_ndt_localizer_main.cpp)
autoware_set_compile_options(${P2D_NDT_NODE})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(NDT_NODES_TEST ndt_nodes_gtest)

  ament_add_gtest(${NDT_NODES_TEST}
          test/test_map_publisher.cpp)
  target_link_libraries(${NDT_NODES_TEST}
    ${PROJECT_NAME}
    ${GeographicLib_LIBRARIES}
    ${YAML_CPP_LIBRARIES})
endif()

# required for tf2
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

target_compile_options(${MAP_PUBLISHER_EXE} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

target_compile_options(${P2D_NDT_NODE} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

ament_auto_package()
