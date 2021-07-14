# Copyright 2021 Tier IV, Inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(outlier_filter)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(PCL 1.8 REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

set(OUTLIER_FILTER_LIB_SRC
  src/radius_search_2d_filter.cpp
  src/voxel_grid_outlier_filter.cpp
)

set(OUTLIER_FILTER_LIB_HEADERS
  include/outlier_filter/radius_search_2d_filter.hpp
  include/outlier_filter/voxel_grid_outlier_filter.hpp
  include/outlier_filter/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OUTLIER_FILTER_LIB_SRC}
  ${OUTLIER_FILTER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES
    test/test_radius_search_2d_filter.cpp
    test/test_voxel_grid_outlier_filter.cpp
    test/outlier_filter_test_utils.hpp)
  set(TEST_OUTLIER_FILTER_EXE test_outlier_filter)
  ament_add_gtest(${TEST_OUTLIER_FILTER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_OUTLIER_FILTER_EXE})
  target_link_libraries(${TEST_OUTLIER_FILTER_EXE} ${PROJECT_NAME})
  ament_target_dependencies(${TEST_OUTLIER_FILTER_EXE} "lidar_utils")
endif()

# ament package generation and installing
ament_export_include_directories(${PCL_INCLUDE_DIRS})
ament_auto_package(INSTALL_TO_SHARE)
