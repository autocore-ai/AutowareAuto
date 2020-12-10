cmake_minimum_required(VERSION 3.6)

project(parking_planner_nodes)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PARKING_PLANNER_NODE_LIB parking_planner_node)
ament_auto_add_library(${PARKING_PLANNER_NODE_LIB} SHARED
  src/parking_planner_nodes/parking_planner_node.cpp)
target_include_directories(${PARKING_PLANNER_NODE_LIB} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS})
autoware_set_compile_options(${PARKING_PLANNER_NODE_LIB})

rclcpp_components_register_node(${PARKING_PLANNER_NODE_LIB}
  PLUGIN "autoware::motion::planning::parking_planner_nodes::ParkingPlannerNode"
  EXECUTABLE ${PARKING_PLANNER_NODE_LIB}_exe
)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(${PARKING_PLANNER_NODE_LIB}_unit_tests test/sanity_checks.cpp)
  target_link_libraries(${PARKING_PLANNER_NODE_LIB}_unit_tests ${PARKING_PLANNER_NODE_LIB} gmp mpfr)
  target_include_directories(${PARKING_PLANNER_NODE_LIB}_unit_tests PRIVATE "include")
  add_dependencies(${PARKING_PLANNER_NODE_LIB}_unit_tests ${PARKING_PLANNER_NODE_LIB})
endif()

ament_auto_package(INSTALL_TO_SHARE param launch)
