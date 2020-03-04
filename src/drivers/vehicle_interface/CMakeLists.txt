cmake_minimum_required(VERSION 3.5)
project(vehicle_interface)
set(CMAKE_CXX_STANDARD 17) # For variant, optional
# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
ament_auto_find_build_dependencies()
# Library
ament_auto_add_library(vehicle_interface
  src/platform_interface.cpp
  src/safety_state_machine.cpp
  src/vehicle_interface_node.cpp)
autoware_set_compile_options(vehicle_interface)

if(BUILD_TESTING)
  # Lint
  autoware_static_code_analysis()
  # Test
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(vehicle_interface_test
    test/gtest_main.cpp
    test/error_handling.cpp
    test/filtering.cpp
    test/sanity_checks.cpp
    test/state_machine.hpp
    test/state_machine.cpp
    test/state_machine_bad_state.cpp
    test/state_machine_clamp.cpp
    test/state_machine_gear.cpp
    test/state_machine_headlight.cpp
    test/state_machine_node.cpp
    test/test_vi_node.hpp)
  target_include_directories(vehicle_interface_test PUBLIC include)
  target_link_libraries(vehicle_interface_test vehicle_interface)
  ament_target_dependencies(vehicle_interface_test
    "autoware_auto_msgs"
    "rclcpp"
    "reference_tracking_controller"
    "signal_filters")
endif()

ament_auto_package()
