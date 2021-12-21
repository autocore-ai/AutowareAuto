# Copyright 2020-2021 Arm Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.12)
project(benchmark_tool)

### Dependencies

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Python3 REQUIRED COMPONENTS Development)

### Build

ament_auto_add_library(kittiobjeval SHARED
  ${PROJECT_NAME}/kittiobjdetsdk/src/evaluate_object.cpp
  ${PROJECT_NAME}/kittiobjdetsdk/src/kittiobjevalmodule.cpp
)
set_target_properties(kittiobjeval PROPERTIES PREFIX "")
target_include_directories(kittiobjeval PRIVATE
  ${Python3_INCLUDE_DIRS}
  ${PROJECT_NAME}/kittiobjdetsdk/include/
)
autoware_set_compile_options(kittiobjeval)

# Prevent errors in external include
if(CMAKE_COMPILER_IS_GNUCXX)
  target_compile_options(kittiobjeval
    PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -Wno-useless-cast>)
endif()

### Install

ament_python_install_package(${PROJECT_NAME})

_ament_cmake_python_get_python_install_dir()
install(TARGETS kittiobjeval
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/kittiobjdetsdk
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  install(DIRECTORY artifacts DESTINATION ${CMAKE_BINARY_DIR})
  ament_add_gtest(test_benchmark_tool test/test_benchmark_tool.cpp)
  autoware_set_compile_options(test_benchmark_tool)
  if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(test_benchmark_tool
      PUBLIC $<$<COMPILE_LANGUAGE:CXX>: -Wno-useless-cast>)
  endif()
  target_link_libraries(test_benchmark_tool kittiobjeval ${Python3_LIBRARIES})
  target_include_directories(test_benchmark_tool PRIVATE
    ${PROJECT_NAME}/kittiobjdetsdk/include/)
  ament_target_dependencies(test_benchmark_tool "autoware_auto_common")
endif()

ament_auto_package()
