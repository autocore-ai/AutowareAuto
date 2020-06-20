cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_algorithm)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_quick_sort_iterative
    test/src/test_quick_sort_iterative.cpp
  )
  target_include_directories(test_quick_sort_iterative
    PRIVATE "include"
  )
endif()

# Ament Exporting
ament_auto_package()
