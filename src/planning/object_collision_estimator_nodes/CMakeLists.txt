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

project(object_collision_estimator_nodes)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# List sources
set(OBJECT_COLLISION_ESTIMATOR_NODE_SRC
  src/object_collision_estimator_node.cpp
)

# List headers
set(OBJECT_COLLISION_ESTIMATOR_NODE_HEADERS
  include/object_collision_estimator_nodes/object_collision_estimator_node.hpp
  include/object_collision_estimator_nodes/visibility_control.hpp
  include/object_collision_estimator_nodes/visualize.hpp
)

# Create a library
set(OBJECT_COLLISION_ESTIMATOR_NODE_LIB object_collision_estimator_node)
ament_auto_add_library(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB} SHARED
  ${OBJECT_COLLISION_ESTIMATOR_NODE_SRC}
  ${OBJECT_COLLISION_ESTIMATOR_NODE_HEADERS}
)
target_include_directories(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
autoware_set_compile_options(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB})

# Create the node executable linking the library
rclcpp_components_register_node(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB}
  PLUGIN "motion::planning::object_collision_estimator_nodes::ObjectCollisionEstimatorNode"
  EXECUTABLE ${OBJECT_COLLISION_ESTIMATOR_NODE_LIB}_exe
)

# Build tests
if(BUILD_TESTING)
  # Enable code linting
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/object_collision_estimator_node_launch.test.py
    TIMEOUT "30"
  )

  # Build unit test
  ament_add_gtest(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB}_unit_tests
    test/${OBJECT_COLLISION_ESTIMATOR_NODE_LIB}_test.cpp)
  autoware_set_compile_options(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB}_unit_tests)
  target_compile_options(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB}_unit_tests PRIVATE -Wno-conversion)
  target_link_libraries(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB}_unit_tests ${OBJECT_COLLISION_ESTIMATOR_NODE_LIB})
  target_include_directories(${OBJECT_COLLISION_ESTIMATOR_NODE_LIB}_unit_tests PRIVATE "include")
endif()

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE param launch)
