cmake_minimum_required(VERSION 3.6)

project(recordreplay_planner_nodes)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(RECORDREPLAY_PLANNER_NODE_LIB recordreplay_planner_node)
ament_auto_add_library(${RECORDREPLAY_PLANNER_NODE_LIB} SHARED
  src/recordreplay_planner_nodes/recordreplay_planner_node.cpp)
autoware_set_compile_options(${RECORDREPLAY_PLANNER_NODE_LIB})

rclcpp_components_register_node(${RECORDREPLAY_PLANNER_NODE_LIB}
  PLUGIN "motion::planning::recordreplay_planner_nodes::RecordReplayPlannerNode"
  EXECUTABLE ${RECORDREPLAY_PLANNER_NODE_LIB}_exe
)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/recordreplay_planner_node_launch.test.py
    TIMEOUT "30"
  )
  # Unit test
  ament_add_gtest(${RECORDREPLAY_PLANNER_NODE_LIB}_unit_tests
    test/gtest_main.cpp
    test/sanity_checks.cpp)
  autoware_set_compile_options(${RECORDREPLAY_PLANNER_NODE_LIB}_unit_tests)
  target_link_libraries(${RECORDREPLAY_PLANNER_NODE_LIB}_unit_tests ${RECORDREPLAY_PLANNER_NODE_LIB})
  target_include_directories(${RECORDREPLAY_PLANNER_NODE_LIB}_unit_tests PRIVATE "include")
endif()

ament_auto_package(INSTALL_TO_SHARE param launch)
