cmake_minimum_required(VERSION 3.5)
project(lanelet2_global_planner_nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# build
set(GB_PLANNER_NODE_LIB lanelet2_global_planner_node)
ament_auto_add_library(${GB_PLANNER_NODE_LIB} SHARED
  include/lanelet2_global_planner_nodes/lanelet2_global_planner_node.hpp
  include/lanelet2_global_planner_nodes/visibility_control.hpp
  src/lanelet2_global_planner_node.cpp
)
target_include_directories(${GB_PLANNER_NODE_LIB} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})

set(GB_PLANNER_NODE_EXE ${GB_PLANNER_NODE_LIB}_exe)
rclcpp_components_register_node(${GB_PLANNER_NODE_LIB}
  PLUGIN "autoware::motion::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode"
  EXECUTABLE ${GB_PLANNER_NODE_EXE}
)

# Ament Exporting
target_include_directories(${GB_PLANNER_NODE_LIB} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
target_compile_options(${GB_PLANNER_NODE_LIB} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

target_include_directories(${GB_PLANNER_NODE_EXE} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
target_compile_options(${GB_PLANNER_NODE_EXE} PRIVATE -Wno-sign-conversion -Wno-conversion -Wno-old-style-cast
        -Wno-useless-cast -Wno-double-promotion -Wno-nonnull-compare -Wuseless-cast)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
