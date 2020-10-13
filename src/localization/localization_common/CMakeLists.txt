# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(localization_common)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(LOCALIZATION_COMMON_LIB_SRC
    src/initialization.cpp
    src/localizer_base.cpp
)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_COMMON_LIB_SRC}
)
# TODO(igor): Remove this once tf2 warnings are removed.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(LOCALIZATION_COMMON_TEST localization_common_gtest)

  ament_add_gtest(${LOCALIZATION_COMMON_TEST}
    test/test_initialization.hpp
    test/test_initialization.cpp
    test/test_relative_localizer.hpp
    test/test_relative_localizer.cpp)

  target_link_libraries(${LOCALIZATION_COMMON_TEST} ${PROJECT_NAME})
  ament_target_dependencies(${LOCALIZATION_COMMON_TEST} "time_utils")
endif()

ament_auto_package()
