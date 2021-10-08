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

project(lonely_world_prediction)

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(LONELY_WORLD_PREDICTION_LIB_SRC
  src/init_from_tracked.cpp
  src/lonely_world_prediction.cpp
  src/make_inputs.cpp
  src/parameters.cpp
)

set(LONELY_WORLD_PREDICTION_LIB_HEADERS
  include/lonely_world_prediction/init_from_tracked.hpp
  include/lonely_world_prediction/lonely_world_prediction.hpp
  include/lonely_world_prediction/test/make_inputs.hpp
  include/lonely_world_prediction/parameters.hpp
  include/lonely_world_prediction/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${LONELY_WORLD_PREDICTION_LIB_SRC}
  ${LONELY_WORLD_PREDICTION_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_lonely_world_prediction.cpp)
  set(TEST_LONELY_WORLD_PREDICTION_EXE test_lonely_world_prediction)
  ament_add_gmock(${TEST_LONELY_WORLD_PREDICTION_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_LONELY_WORLD_PREDICTION_EXE})
  # needed to build gmock with autoware compile options
  target_compile_options(${TEST_LONELY_WORLD_PREDICTION_EXE} PUBLIC
    -Wno-sign-conversion
  )
  target_link_libraries(${TEST_LONELY_WORLD_PREDICTION_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE
  launch
)
