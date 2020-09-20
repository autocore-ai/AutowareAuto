# Copyright 2020 Arm Limited
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

cmake_minimum_required(VERSION 3.5)

project(object_collision_estimator_node)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# List sources
set(OBJECT_COLLISION_ESTIMATOR_NODE_SRC
  src/object_collision_estimator_node.cpp
)

# List headers
set(OBJECT_COLLISION_ESTIMATOR_NODE_HEADERS
  include/object_collision_estimator_node/object_collision_estimator_node.hpp
  include/object_collision_estimator_node/visibility_control.hpp
)

# Create a library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OBJECT_COLLISION_ESTIMATOR_NODE_SRC}
  ${OBJECT_COLLISION_ESTIMATOR_NODE_HEADERS}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
autoware_set_compile_options(${PROJECT_NAME})

# Create the node executable linking the library
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "motion::planning::object_collision_estimator_node::ObjectCollisionEstimatorNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

# Build tests
if(BUILD_TESTING)
  # Enable code linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Build unit test
  ament_add_gtest(${PROJECT_NAME}_unit_tests
    test/${PROJECT_NAME}_test.cpp)
  target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE "include")
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE param launch)
