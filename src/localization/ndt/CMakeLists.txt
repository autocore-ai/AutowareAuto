# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(ndt)

#dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(localization_common REQUIRED)
find_package(optimization REQUIRED)
find_package(lidar_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# includes
include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

set(NDT_NODES_LIB_SRC
    src/ndt.cpp
    src/ndt_map.cpp
)

set(NDT_NODES_LIB_HEADERS
    include/ndt/visibility_control.hpp
    include/ndt/ndt_representations.hpp
    include/ndt/ndt_optimization_problem.hpp
    include/ndt/ndt_map.hpp
    include/ndt/ndt_localizer.hpp)

add_library(
${PROJECT_NAME} SHARED
        ${NDT_NODES_LIB_SRC}
        ${NDT_NODES_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} "localization_common" "optimization" "lidar_utils"
        "sensor_msgs" "geometry_msgs" "eigen3")
# point_cloud2_iterator headers need this suprression
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion)


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


autoware_install(HAS_INCLUDE)
ament_package(
                    CONFIG_EXTRAS_POST "ndt-extras.cmake"
)
