cmake_minimum_required(VERSION 3.5)
project(lanelet2_global_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# build
set(GB_PLANNER_LIB lanelet2_global_planner)
ament_auto_add_library(${GB_PLANNER_LIB} SHARED
  include/lanelet2_global_planner/lanelet2_global_planner.hpp
  src/lanelet2_global_planner.cpp
  include/lanelet2_global_planner/visibility_control.hpp
)

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(GB_PLANNER_GTEST lanelet2_global_planner_gtest)
  ament_add_gtest(${GB_PLANNER_GTEST}
          test/test_lanelet2_global_planner.hpp
          test/test_lanelet2_global_planner.cpp
          include/lanelet2_global_planner/lanelet2_global_planner.hpp
          src/lanelet2_global_planner.cpp
  )

  target_link_libraries(${GB_PLANNER_GTEST} ${GB_PLANNER_LIB})
  target_include_directories(${GB_PLANNER_GTEST} PRIVATE "test/include" "include")

  target_compile_options(${GB_PLANNER_GTEST} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)
endif()

# Ament Exporting
target_include_directories(${GB_PLANNER_LIB} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
target_compile_options(${GB_PLANNER_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

ament_auto_package()
