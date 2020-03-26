# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(ndt_nodes)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS io)
ament_auto_find_build_dependencies()

# includes
include_directories(include)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

set(NDT_NODES_LIB_SRC
    src/map_publisher.cpp
)

set(NDT_NODES_LIB_HEADERS
    include/ndt_nodes/visibility_control.hpp
    include/ndt_nodes/map_publisher.hpp
    include/ndt_nodes/ndt_localizer_nodes.hpp)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${NDT_NODES_LIB_SRC}
        ${NDT_NODES_LIB_HEADERS}
)

autoware_set_compile_options(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

set(MAP_PUBLISHER_EXE ndt_map_publisher_exe)
ament_auto_add_executable(${MAP_PUBLISHER_EXE} src/map_publisher_main.cpp)
autoware_set_compile_options(${MAP_PUBLISHER_EXE})

set(P2D_NDT_NODE p2d_ndt_localizer_exe)
ament_auto_add_executable(${P2D_NDT_NODE} src/p2d_ndt_localizer_main.cpp)
autoware_set_compile_options(${P2D_NDT_NODE})

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  # gtest
  set(NDT_NODES_TEST ndt_nodes_gtest)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${NDT_NODES_TEST}
          test/test_map_publisher.cpp)
  target_link_libraries(${NDT_NODES_TEST} ${PROJECT_NAME})
  ament_target_dependencies(${NDT_NODES_TEST} ${PROJECT_NAME})
endif()

# required for tf2
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

target_compile_options(${MAP_PUBLISHER_EXE} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

target_compile_options(${P2D_NDT_NODE} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
