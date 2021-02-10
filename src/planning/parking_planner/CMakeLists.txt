# Copyright 2020-2021 Embotech AG, Arm Limited
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

### Export headers
project(parking_planner)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(casadi REQUIRED)
ament_auto_find_build_dependencies()

# Temporarily override build type - See #696
set(CMAKE_BUILD_TYPE "")

# --- Add a solver generator executable
# This has to be built first, then run, and only _then_ can the bicycle planner
# library be built - due to the code generation. This is implemented by making it
# a dependency of the parking_planner target.
#
# The solver generator creates two things:
# - a header that is to be included by the bicycle planner library
# - a C file that is to be built into its own shared library for use
#   by the motion planner at runtime.
#
# The library gets loaded by the code itself, it hence does not have to be
# linked, but available to load at runtime.
ament_auto_add_executable(${PROJECT_NAME}_solver_generator
    src/generate_nlp_planner_solver.cpp)
target_include_directories(${PROJECT_NAME}_solver_generator PRIVATE "include")
target_link_libraries(${PROJECT_NAME}_solver_generator casadi)
autoware_set_compile_options(${PROJECT_NAME}_solver_generator)

# --- Add a post-build command to run the solver generator
# We store the generation results in the directory given in the variable
# GENERATED_PATH. This is later added to the includes of the normal
# parking planner library.
set(GENERATED_PATH ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_generated)
add_custom_command(
    DEPENDS ${PROJECT_NAME}_solver_generator
    OUTPUT ${GENERATED_PATH}/parking_planner_callbacks.c
    COMMAND
    mkdir -p ${GENERATED_PATH} && cd ${GENERATED_PATH} &&
    ${CMAKE_BINARY_DIR}/${PROJECT_NAME}_solver_generator ${CMAKE_INSTALL_PREFIX}/lib)

# Add a shim target so our callback builder target can depend
# properly on the solver generator outputs
add_custom_target(${PROJECT_NAME}_generator_wrapper
    DEPENDS ${GENERATED_PATH}/parking_planner_callbacks.c)

# Add the actual "make a shared library from the codegen'ed code" target
ament_auto_add_library(${PROJECT_NAME}_callbacks SHARED
    ${GENERATED_PATH}/parking_planner_callbacks.c)
add_dependencies(${PROJECT_NAME}_callbacks ${PROJECT_NAME}_generator_wrapper)


# Target for the main planner library
ament_auto_add_library(${PROJECT_NAME} SHARED
    src/astar_path_planner.cpp
    src/nlp_path_planner.cpp
    src/parking_planner.cpp)
target_link_libraries(${PROJECT_NAME} casadi gmp mpfr)
target_include_directories(${PROJECT_NAME} PRIVATE ${GENERATED_PATH})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_solver_generator)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run common linters
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  # If enabled, turn on plotting for integration tests
  option(PARKING_PLANNER_TEST_PLOTS "Enable plots for parking planner tests" OFF)
  if(PARKING_PLANNER_TEST_PLOTS)
    # Test plots are enabled, look for python and download matplotlib-cpp
    find_package(PythonLibs)
    file(DOWNLOAD
      https://raw.githubusercontent.com/lava/matplotlib-cpp/823de00fbae746c09c11efea59a2738da44518b8/matplotlibcpp.h
      "${GENERATED_PATH}/matplotlibcpp.h")

    # ... and tell the tests about it by defining a preprocessor constant
    add_definitions(-DPARKING_PLANNER_TEST_PLOTS)
  endif()

  # gtest
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
      test/gtest_main.cpp
      test/test_astar.cpp
      test/test_basics.cpp
      test/test_geometry.cpp
      test/test_integration.cpp
      test/test_nlp.cpp)
  target_include_directories(${PROJECT_NAME}_test PRIVATE "include")
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  if(PARKING_PLANNER_TEST_PLOTS)
    target_include_directories(${PROJECT_NAME}_test PRIVATE ${PYTHON_INCLUDE_PATH})
    target_include_directories(${PROJECT_NAME}_test PRIVATE ${GENERATED_PATH})
    target_link_libraries(${PROJECT_NAME}_test ${PYTHON_LIBRARIES})
  endif()
endif()

# Ament Exporting
ament_auto_package()
