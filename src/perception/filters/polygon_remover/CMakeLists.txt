# Copyright 2021 The Autoware Foundation
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

project(polygon_remover)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
ament_auto_find_build_dependencies()

set(POLYGON_REMOVER_LIB_SRC
  src/polygon_remover.cpp
)

set(POLYGON_REMOVER_LIB_HEADERS
  include/polygon_remover/polygon_remover.hpp
  include/polygon_remover/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${POLYGON_REMOVER_LIB_SRC}
  ${POLYGON_REMOVER_LIB_HEADERS}
)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
target_link_libraries(${PROJECT_NAME} CGAL CGAL::CGAL CGAL::CGAL_Core)
autoware_set_compile_options(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_polygon_remover.cpp)
  set(TEST_POLYGON_REMOVER_EXE test_polygon_remover)
  ament_add_gtest(${TEST_POLYGON_REMOVER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_POLYGON_REMOVER_EXE})
  target_link_libraries(${TEST_POLYGON_REMOVER_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
