cmake_minimum_required(VERSION 3.5)
project(lanelet2_global_planner)

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
autoware_set_compile_options(${GB_PLANNER_LIB})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  # Install test maps to share directory
  install(DIRECTORY test/map_data
    DESTINATION share/${PROJECT_NAME}/test
  )

  # gtest
  set(GB_PLANNER_GTEST lanelet2_global_planner_gtest)
  ament_add_gtest(${GB_PLANNER_GTEST}
          test/test_lanelet2_global_planner.hpp
          test/test_lanelet2_global_planner.cpp
          include/lanelet2_global_planner/lanelet2_global_planner.hpp
          src/lanelet2_global_planner.cpp)
  ament_target_dependencies(${GB_PLANNER_GTEST} "ament_index_cpp")
  autoware_set_compile_options(${GB_PLANNER_GTEST})
  target_compile_options(${GB_PLANNER_GTEST} PRIVATE -Wno-sign-conversion -Wno-conversion
    -Wno-useless-cast -Wno-double-promotion)
  target_link_libraries(${GB_PLANNER_GTEST} ${GB_PLANNER_LIB})
  target_include_directories(${GB_PLANNER_GTEST} PRIVATE "test/include" "include")
endif()

# Ament Exporting

ament_auto_package()
