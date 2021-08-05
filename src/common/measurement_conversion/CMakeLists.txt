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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

cmake_minimum_required(VERSION 3.5)

project(measurement_conversion)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(MEASUREMENT_CONVERSION_LIB_SRC
  src/measurement_conversion.cpp
  src/measurement_transformation.cpp
)

set(MEASUREMENT_CONVERSION_LIB_HEADERS
  include/measurement_conversion/eigen_utils.hpp
  include/measurement_conversion/measurement_conversion.hpp
  include/measurement_conversion/measurement_transformation.hpp
  include/measurement_conversion/measurement_typedefs.hpp
  include/measurement_conversion/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${MEASUREMENT_CONVERSION_LIB_SRC}
  ${MEASUREMENT_CONVERSION_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_cppcheck has an internal error for no good reason.
  set(ament_cmake_cppcheck_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_measurement_conversion.cpp test/test_eigen_utils.cpp)
  set(TEST_MEASUREMENT_CONVERSION_EXE test_measurement_conversion)
  ament_add_gtest(${TEST_MEASUREMENT_CONVERSION_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_MEASUREMENT_CONVERSION_EXE})
  target_link_libraries(${TEST_MEASUREMENT_CONVERSION_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
