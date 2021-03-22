cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_algorithm)

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
  autoware_set_compile_options(test_quick_sort_iterative)
  target_include_directories(test_quick_sort_iterative
    PRIVATE "include"
  )
endif()

# Ament Exporting
ament_auto_package()
