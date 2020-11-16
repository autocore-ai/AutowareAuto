cmake_minimum_required(VERSION 3.5)
project(joystick_vehicle_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# Node library
ament_auto_add_library(joystick_vehicle_interface SHARED
  src/joystick_vehicle_interface.cpp
  include/joystick_vehicle_interface/joystick_vehicle_interface.hpp)
autoware_set_compile_options(joystick_vehicle_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
