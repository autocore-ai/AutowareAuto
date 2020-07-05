cmake_minimum_required(VERSION 3.5)
project(rosbag2_moriyama_converter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(dataset_converter_node SHARED src/dataset_converter.cpp)
autoware_set_compile_options(dataset_converter_node)


rclcpp_components_register_node(dataset_converter_node
  PLUGIN "rosbag2_moriyama_converter::DatasetConverterNode"
  EXECUTABLE dataset_converter
)

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
