# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(optimization)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# includes
include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})


set(OPTIMIZATION_LIB_SRC
        src/newtons_method_optimizer.cpp
        src/optimizer_options.cpp
        src/utils.cpp)

set(OPTIMIZATION_LIB_HEADERS
        include/optimization/visibility_control.hpp
        include/optimization/utils.hpp
        include/optimization/optimization_problem.hpp
        include/optimization/optimizer_options.hpp
        include/optimization/optimizer.hpp
        include/optimization/newtons_method_optimizer.hpp
        include/optimization/line_search/line_search.hpp
        include/optimization/line_search/fixed_line_search.hpp
        include/optimization/line_search/more_thuente_line_search.hpp)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${OPTIMIZATION_LIB_HEADERS}
        ${OPTIMIZATION_LIB_SRC}
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  # gtest
  set(OPTIMIZATION_TEST optimization_gtest)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${OPTIMIZATION_TEST}
          test/test_newton_optimization.hpp
          test/test_newton_optimization.cpp
          test/test_cache_states.hpp
          test/test_cache_states.cpp)
  target_link_libraries(${OPTIMIZATION_TEST} ${PROJECT_NAME})
  ament_target_dependencies(${OPTIMIZATION_TEST} ${PROJECT_NAME})
endif()

ament_auto_package()
