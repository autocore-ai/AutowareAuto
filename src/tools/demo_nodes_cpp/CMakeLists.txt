cmake_minimum_required(VERSION 3.5)

project(demo_nodes_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "rcutils"
    "std_msgs")
  install(TARGETS ${target}
  DESTINATION lib/${PROJECT_NAME})
endfunction()

# Tutorials of Publish/Subscribe with Topics
custom_executable(topics talker)
custom_executable(topics listener)

if(BUILD_TESTING)
#TODO: include other linterns
find_package(integration_tests REQUIRED)
integration_tests(
  EXPECTED_OUTPUT_DIR "test/expected_outputs"
  COMMANDS
  "talker"
)
endif()

ament_package()
