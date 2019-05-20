cmake_minimum_required(VERSION 3.5)

### Export headers
project(geometry)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_convex_hull_gtest
    test/src/test_convex_hull.cpp
  )
  target_include_directories(test_convex_hull_gtest
    PRIVATE "test/include" "include"
  )
  ament_target_dependencies(test_convex_hull_gtest
    "geometry_msgs"
  )
endif()

# Ament Exporting
autoware_install(HAS_INCLUDE)
ament_package()
