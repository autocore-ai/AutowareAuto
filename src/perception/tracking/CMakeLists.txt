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

project(tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

set(ASSOCIATION_LIB_SRC
  src/data_association.cpp
)

set(ASSOCIATION_LIB_HEADERS
  include/tracking/data_association.hpp
  include/tracking/visibility_control.hpp
)

# generate library
set(ASSOCIATION_LIB data_association)
ament_auto_add_library(${ASSOCIATION_LIB} SHARED
  ${ASSOCIATION_LIB_SRC}
  ${ASSOCIATION_LIB_HEADERS}
)
autoware_set_compile_options(${ASSOCIATION_LIB})
ament_target_dependencies(${ASSOCIATION_LIB} Eigen3)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_data_association.cpp)
  set(TEST_ASSOCIATION_EXE data_association_gtest)
  ament_add_gtest(${TEST_ASSOCIATION_EXE} ${TEST_SOURCES})
  target_include_directories(${TEST_ASSOCIATION_EXE} PRIVATE include)
  autoware_set_compile_options(${TEST_ASSOCIATION_EXE})
  target_link_libraries(${TEST_ASSOCIATION_EXE} ${ASSOCIATION_LIB})
endif()

# ament package generation and installing
ament_auto_package()
