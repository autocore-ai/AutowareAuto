# Copyright 2017-2019 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
project(euclidean_cluster)

## dependencies
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_geometry REQUIRED)
find_package(lidar_utils REQUIRED)
find_package(autoware_auto_msgs REQUIRED)

# includes
add_library(${PROJECT_NAME} SHARED
  include/euclidean_cluster/euclidean_cluster.hpp
  include/euclidean_cluster/visibility_control.hpp
  src/euclidean_cluster.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC "include")
autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  "autoware_auto_geometry"
  "lidar_utils"
  "autoware_auto_msgs")
target_compile_options(${PROJECT_NAME} PUBLIC "-O0" PRIVATE -Wno-sign-conversion -Wno-conversion)

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()
  find_package(ament_cmake_pclint REQUIRED)
  ament_pclint(CHECK_HEADERS)

  # gtest
  set(EUCLIDEAN_CLUSTER_GTEST euclidean_cluster_gtest)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${EUCLIDEAN_CLUSTER_GTEST} test/src/test_euclidean_cluster.cpp)
  target_include_directories(${EUCLIDEAN_CLUSTER_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${EUCLIDEAN_CLUSTER_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${EUCLIDEAN_CLUSTER_GTEST} "autoware_auto_geometry")
endif()

autoware_install(LIBRARIES ${PROJECT_NAME} HAS_INCLUDE)
# Ament Exporting
ament_export_dependencies(
  "autoware_auto_geometry"
  "lidar_utils"
  "autoware_auto_msgs")
ament_package()
