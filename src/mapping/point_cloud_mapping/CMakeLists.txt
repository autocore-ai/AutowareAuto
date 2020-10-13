# Copyright 2020 Apex.AI, Inc.
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
project(point_cloud_mapping)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS io)
ament_auto_find_build_dependencies()

# includes
include_directories(include ${PCL_INCLUDE_DIRS})

set(PC_MAPPING_SRC
    src/map.cpp
    src/point_cloud_mapper.cpp)

set(PC_MAPPING_HEADERS
    include/point_cloud_mapping/visibility_control.hpp
    include/point_cloud_mapping/policies.hpp
    include/point_cloud_mapping/map.hpp
    include/point_cloud_mapping/point_cloud_mapper.hpp)

ament_auto_add_library(
${PROJECT_NAME} SHARED
        ${PC_MAPPING_HEADERS}
        ${PC_MAPPING_SRC}
)

autoware_set_compile_options(${PROJECT_NAME})
# turn off warnings to be able to successfully compile upstream ros packages
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
set(ROS_NO_WARN_LIST
        -Wno-sign-conversion
        -Wno-conversion
        -Wno-old-style-cast
        -Wno-useless-cast
        -Wno-double-promotion
        -Wno-nonnull-compare
        -Wuseless-cast)
target_compile_options(${PROJECT_NAME} PRIVATE ${ROS_NO_WARN_LIST})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(PC_MAPPING_TEST pcm_test)

  ament_add_gtest(${PC_MAPPING_TEST}
          test/test_pc_mapper.hpp
          test/test_pc_mapper.cpp
          test/test_map.hpp
          test/test_map.cpp
  )

  target_link_libraries(${PC_MAPPING_TEST} ${PROJECT_NAME})
  autoware_set_compile_options(${PC_MAPPING_TEST})
  # turn off warnings to be able to successfully compile upstream ros packages
  target_compile_options(${PC_MAPPING_TEST} PRIVATE ${ROS_NO_WARN_LIST})
endif()
ament_export_include_directories(${PCL_INCLUDE_DIRS})
ament_export_libraries(${PCL_LIBRARIES})
ament_auto_package()
