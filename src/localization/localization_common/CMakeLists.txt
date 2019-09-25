# Copyright 2019 Apex.AI, Inc.
# All rights reserved.

cmake_minimum_required(VERSION 3.5)
project(localization_common)

#dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(tf2)
find_package(geometry_msgs)
# includes
include_directories(include)


set(LOCALIZATION_COMMON_LIB_SRC
    src/localizer_base.cpp
)

set(LOCALIZATION_COMMON_LIB_HEADERS
    include/localization_common/visibility_control.hpp
    include/localization_common/initialization.hpp
    include/localization_common/localizer_base.hpp)

add_library(
${PROJECT_NAME} SHARED
        ${LOCALIZATION_COMMON_LIB_SRC}
        ${LOCALIZATION_COMMON_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

ament_target_dependencies(${PROJECT_NAME} "tf2" "geometry_msgs")

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

autoware_install(HAS_INCLUDE)
ament_package()
