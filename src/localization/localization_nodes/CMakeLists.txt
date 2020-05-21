# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(localization_nodes)

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

# includes
set(LOCALIZATION_NODES_LIB_SRC
    src/localization_node.cpp
)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_NODES_LIB_SRC}
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(LOCALIZATION_NOTE_TEST localization_node_gtest)

  ament_add_gtest(${LOCALIZATION_NOTE_TEST}
          test/test_relative_localizer_node.hpp
          test/test_relative_localizer_node.cpp)

  target_link_libraries(${LOCALIZATION_NOTE_TEST} ${PROJECT_NAME})
endif()


target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

ament_auto_package()
