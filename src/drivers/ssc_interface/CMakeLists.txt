# Copyright 2020 The Autoware Foundation
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

project(ssc_interface)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(SSC_INTERFACE_SRC
  src/ssc_interface.cpp
  src/ssc_interface_node.cpp
)

set(SSC_INTERFACE_HEADERS
  include/ssc_interface/ssc_interface.hpp
  include/ssc_interface/ssc_interface_node.hpp
  include/ssc_interface/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SSC_INTERFACE_SRC}
  ${SSC_INTERFACE_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

set(NODE_NAME ${PROJECT_NAME}_node_exe)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ssc_interface::SscInterfaceNode"
  EXECUTABLE ${NODE_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${NODE_NAME})

  # Unit tests
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_ssc_interface.cpp
  )
  autoware_set_compile_options(test_${PROJECT_NAME})
  target_compile_options(test_${PROJECT_NAME} PRIVATE -Wno-double-promotion -Wno-float-conversion)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  add_dependencies(test_${PROJECT_NAME} ${PROJECT_NAME})
  ament_target_dependencies(test_${PROJECT_NAME}
    automotive_platform_msgs
  )
endif()

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    param
)
