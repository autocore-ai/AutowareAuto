cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_algorithm)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_quick_sort_iterative
    test/src/test_quick_sort_iterative.cpp
  )
  target_include_directories(test_quick_sort_iterative
    PRIVATE "include"
  )
endif()

# Ament Exporting
autoware_install(HAS_INCLUDE)
ament_package()