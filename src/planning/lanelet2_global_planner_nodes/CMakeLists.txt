cmake_minimum_required(VERSION 3.5)
project(lanelet2_global_planner_nodes)

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
autoware_set_compile_options(${GB_PLANNER_NODE_LIB})

set(GB_PLANNER_NODE_EXE ${GB_PLANNER_NODE_LIB}_exe)
rclcpp_components_register_node(${GB_PLANNER_NODE_LIB}
  PLUGIN "autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode"
  EXECUTABLE ${GB_PLANNER_NODE_EXE}
)

# Testing
if(BUILD_TESTING)
  ## Linters were previously not part of this package
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/lanelet2_global_planner_node_launch.test.py
    TIMEOUT "30"
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
