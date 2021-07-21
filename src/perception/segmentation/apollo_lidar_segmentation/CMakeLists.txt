# Copyright 2021 Arm Ltd.
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

cmake_minimum_required(VERSION 3.6)
project(apollo_lidar_segmentation)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(tvm_runtime_DIR ${tvm_vendor_DIR})
find_package(tvm_runtime CONFIG REQUIRED)
find_package(PCL 1.10 REQUIRED)

# Get target backend
set(${PROJECT_NAME}_BACKEND llvm CACHE STRING "${PROJECT_NAME} neural network backend")

# Test if files exist. The result is set in baidu_cnn_FOUND
autoware_check_neural_network(baidu_cnn "${${PROJECT_NAME}_BACKEND}")
if(baidu_cnn_FOUND)
  # Library
  ament_auto_add_library(${PROJECT_NAME} SHARED
    include/apollo_lidar_segmentation/apollo_lidar_segmentation.hpp
    include/apollo_lidar_segmentation/cluster2d.hpp
    include/apollo_lidar_segmentation/disjoint_set.hpp
    include/apollo_lidar_segmentation/feature_generator.hpp
    include/apollo_lidar_segmentation/feature_map.hpp
    include/apollo_lidar_segmentation/log_table.hpp
    include/apollo_lidar_segmentation/util.hpp
    src/apollo_lidar_segmentation.cpp
    src/cluster2d.cpp
    src/feature_generator.cpp
    src/feature_map.cpp
    src/log_table.cpp
  )

  autoware_set_compile_options(${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-sign-conversion" "-Wno-conversion")

  target_compile_definitions(${PROJECT_NAME} PRIVATE NETWORKS_BACKEND=${${PROJECT_NAME}_BACKEND})

  # Set includes as "SYSTEM" to avoid compiler errors on their headers.
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    "${PCL_INCLUDE_DIRS}"
    "${tvm_vendor_INCLUDE_DIRS}"
  )

  target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tvm_runtime_LIBRARIES}
  )

  if(BUILD_TESTING)
    # Enable code linting
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # gtest
    set(APOLLO_LIDAR_SEGMENTATION_GTEST apollo_lidar_segmentation_gtest)
    ament_add_gtest(${APOLLO_LIDAR_SEGMENTATION_GTEST} test/main.cpp TIMEOUT 120)
    autoware_set_compile_options(${APOLLO_LIDAR_SEGMENTATION_GTEST})
    target_include_directories(${APOLLO_LIDAR_SEGMENTATION_GTEST} PRIVATE "include")
    target_link_libraries(${APOLLO_LIDAR_SEGMENTATION_GTEST} ${PROJECT_NAME})
  endif()

  ament_export_include_directories(${PCL_INCLUDE_DIRS})
  ament_auto_package()
else()
  message(WARNING "Neural network not found, skipping package.")
endif()
