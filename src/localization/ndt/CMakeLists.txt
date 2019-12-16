# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(ndt)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
ament_auto_find_build_dependencies()

# includes
include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

set(NDT_NODES_LIB_SRC
    src/ndt.cpp
    src/ndt_map.cpp
    src/ndt_voxel.cpp
)

set(NDT_NODES_LIB_HEADERS
    include/ndt/visibility_control.hpp
    include/ndt/ndt_representations.hpp
    include/ndt/ndt_optimization_problem.hpp
    include/ndt/ndt_voxel.hpp
    include/ndt/ndt_map.hpp
    include/ndt/ndt_localizer.hpp)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${NDT_NODES_LIB_SRC}
        ${NDT_NODES_LIB_HEADERS}
)

autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  # gtest
  set(NDT_TEST ndt_gtest)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${NDT_TEST} test/test_ndt_map.hpp
          test/test_ndt_map.cpp)
  target_link_libraries(${NDT_TEST} ${PROJECT_NAME})
  ament_target_dependencies(${NDT_TEST} ${PROJECT_NAME})
endif()

# required for tf2
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)


ament_auto_package()
