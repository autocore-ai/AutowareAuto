cmake_minimum_required(VERSION 3.5)
project(autoware_state_monitor)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build
set(STATE_MONITOR_SRC
  src/autoware_state_monitor_node/autoware_state_monitor_node.cpp
  src/autoware_state_monitor_node/odometry_updater.cpp
  src/autoware_state_monitor_node/state_machine.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${STATE_MONITOR_SRC})
autoware_set_compile_options(${PROJECT_NAME})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::state_monitor::AutowareStateMonitorNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  add_ros_test(
    test/autoware_state_monitor_node_launch.test.py
    TIMEOUT "30"
  )

  ament_add_gtest(${PROJECT_NAME}_test
    test/autoware_state_monitor_node_test.cpp
    test/odometry_updater_test.cpp
    test/state_machine_test.cpp
    test/state_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)