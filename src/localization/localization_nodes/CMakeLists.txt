# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(localization_nodes)

#dependencies
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# includes
include_directories(include)

set(LOCALIZATION_NODES_LIB_SRC
    src/localization_node.cpp
)

set(LOCALIZATION_NODES_LIB_HEADERS
    include/localization_nodes/visibility_control.hpp
    include/localization_nodes/localization_node.hpp)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_NODES_LIB_SRC}
        ${LOCALIZATION_NODES_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  # gtest
  set(LOCALIZATION_NOTE_TEST localization_node_gtest)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${LOCALIZATION_NOTE_TEST}
          test/test_relative_localizer_node.hpp
          test/test_relative_localizer_node.cpp)

  target_link_libraries(${LOCALIZATION_NOTE_TEST} ${PROJECT_NAME})
  ament_target_dependencies(${LOCALIZATION_NOTE_TEST} ${PROJECT_NAME})
endif()


target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

ament_auto_package()
