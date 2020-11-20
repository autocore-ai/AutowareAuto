# Copyright 2021 Arm Limited and Contributors.
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
project(tvm_utility)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TVM_UTILITY_NODE_LIB_HEADERS
  "include/${PROJECT_NAME}/pipeline.hpp"
)

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED ${TVM_UTILITY_NODE_LIB_HEADERS})
autoware_set_compile_options(${PROJECT_NAME})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

if(BUILD_TESTING)
  # Enable code linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # compile each folder inside test/ as a test case
  find_package(ament_cmake_gtest REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(tvm_vendor REQUIRED)
  set(tvm_runtime_DIR ${tvm_vendor_DIR})
  find_package(tvm_runtime CONFIG REQUIRED)

  set(TEST_ARTIFACTS "${CMAKE_CURRENT_LIST_DIR}/artifacts")
  file(GLOB TEST_CASES test/*)
  foreach(TEST_FOLDER ${TEST_CASES})
    # the folder name becomes the test case name
    file(RELATIVE_PATH TEST_CASE_NAME ${CMAKE_CURRENT_LIST_DIR}/test ${TEST_FOLDER})

    if(EXISTS "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/inference_engine_tvm_config.hpp"
      AND EXISTS "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/deploy_lib.so"
      AND EXISTS "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/deploy_graph.json"
      AND EXISTS "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/deploy_param.params"
    )

      # add all cpp files in the folder to the target
      file(GLOB TEST_CASE_SOURCES ${TEST_FOLDER}/*.cpp)
      ament_add_gtest(${TEST_CASE_NAME} ${TEST_CASE_SOURCES})
      ament_target_dependencies(${TEST_CASE_NAME}
        "tvm_vendor"
        "sensor_msgs")

      target_link_libraries("${TEST_CASE_NAME}"
        "${OpenCV_LIBRARIES}"
        "${tvm_runtime_LIBRARIES}"
      )

      target_include_directories("${TEST_CASE_NAME}" SYSTEM PUBLIC
        "${OpenCV_INCLUDE_DIRS}"
        "${tvm_vendor_INCLUDE_DIRS}"
        "include"
        "${TEST_ARTIFACTS}/${TEST_CASE_NAME}"
      )

      file(
        COPY
          "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/inference_engine_tvm_config.hpp"
        DESTINATION
          ${CMAKE_BINARY_DIR})

      file(
        COPY
          "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/deploy_lib.so"
        DESTINATION
          ${CMAKE_BINARY_DIR})

      file(
        COPY
          "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/deploy_graph.json"
        DESTINATION
          ${CMAKE_BINARY_DIR})

      file(
        COPY
          "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/deploy_param.params"
        DESTINATION
          ${CMAKE_BINARY_DIR})

      # Copy yolo_v2_tiny specific files
      if(EXISTS "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/test_image_0.jpg")
        file(
          COPY
            ${TEST_ARTIFACTS}/${TEST_CASE_NAME}/test_image_0.jpg
          DESTINATION
            ${CMAKE_BINARY_DIR})
      endif()

      if(EXISTS "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/anchors.csv")
        file(
          COPY
            ${TEST_ARTIFACTS}/${TEST_CASE_NAME}/anchors.csv
          DESTINATION
            ${CMAKE_BINARY_DIR})
      endif()

      if(EXISTS "${TEST_ARTIFACTS}/${TEST_CASE_NAME}/labels.txt")
        file(
          COPY
            ${TEST_ARTIFACTS}/${TEST_CASE_NAME}/labels.txt
          DESTINATION
            ${CMAKE_BINARY_DIR})
      endif()

    else()
      message(WARNING "No model is generated for ${TEST_FOLDER}, skipping test")
    endif()

  endforeach()
endif()

ament_auto_package()
