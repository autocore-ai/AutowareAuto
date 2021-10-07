cmake_minimum_required(VERSION 3.5)
project(emergency_handler)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build
set(EMERGENCY_HANDLER_SRC
  src/emergency_handler/emergency_handler_node.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED ${EMERGENCY_HANDLER_SRC})
autoware_set_compile_options(${PROJECT_NAME})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::emergency_handler::EmergencyHandlerNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  add_ros_test(
    test/emergency_handler_launch.test.py
    TIMEOUT "30"
  )

  ament_add_gtest(${PROJECT_NAME}_test
    test/emergency_handler_node_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  param
)