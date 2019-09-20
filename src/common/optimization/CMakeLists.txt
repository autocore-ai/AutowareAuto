# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(optimization)

#dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# includes
include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})


set(OPTIMIZATION_LIB_SRC
        src/optimization.cpp
        )

set(OPTIMIZATION_LIB_HEADERS
        include/optimization/visibility_control.hpp
        include/optimization/optimization_problem.hpp
        include/optimization/optimizer.hpp)

add_library(
${PROJECT_NAME} SHARED
        ${OPTIMIZATION_LIB_SRC}
        ${OPTIMIZATION_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} "eigen3")



autoware_install(HAS_INCLUDE)
ament_package(
            CONFIG_EXTRAS_POST "optimization-extras.cmake"
)
