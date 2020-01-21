# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(localization_common)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
ament_auto_find_build_dependencies()
# includes
include_directories(include)


set(LOCALIZATION_COMMON_LIB_SRC
    src/initialization.cpp
    src/localizer_base.cpp
)

set(LOCALIZATION_COMMON_LIB_HEADERS
    include/localization_common/visibility_control.hpp
    include/localization_common/initialization.hpp
    include/localization_common/localizer_base.hpp)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_COMMON_LIB_SRC}
        ${LOCALIZATION_COMMON_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
  # gtest
  set(LOCALIZATION_COMMON_TEST localization_common_gtest)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(time_utils REQUIRED)

  ament_add_gtest(${LOCALIZATION_COMMON_TEST}
          test/test_initialization.hpp
          test/test_initialization.cpp)

  target_link_libraries(${LOCALIZATION_COMMON_TEST} ${PROJECT_NAME})

  ament_target_dependencies(${LOCALIZATION_COMMON_TEST} ${PROJECT_NAME} "time_utils")
endif()

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

ament_auto_package()
