cmake_minimum_required(VERSION 3.5)

project(demo_nodes_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_cmake REQUIRED)

function(custom_executable subfolder target)
  add_executable(${target} src/${subfolder}/${target}.cpp)
  ament_target_dependencies(${target}
    "rclcpp"
    "rcutils"
    "std_msgs")
  autoware_set_compile_options(${target})
  # ROS 2 rclcpp does not have visibility set correctly yet
  # See Apex internal ticket: https://gitlab.apex.ai/ApexAI/grand_central/commit/da8839415dcb7b980566b9bda7d25813a4b5b54b
  # Until then we will be unsetting it explicitly
  target_compile_options(${target} PRIVATE -fvisibility=default)
endfunction()

# Tutorials of Publish/Subscribe with Topics
custom_executable(topics talker)
custom_executable(topics listener)

if(BUILD_TESTING)
  # Linting
  autoware_static_code_analysis()

  file(COPY "${CMAKE_SOURCE_DIR}/test/expected_outputs"
    DESTINATION "${CMAKE_BINARY_DIR}")

#  TODO(esteve): Disabling integration tests for now. Port them to Dashing.
#  See https://gitlab.com/AutowareAuto/AutowareAuto/issues/89
#  find_package(integration_tests REQUIRED)
#    integration_tests(
#    EXPECTED_OUTPUT_DIR "${CMAKE_BINARY_DIR}/expected_outputs"
#    COMMANDS
#    "talker:::listener")
endif()

autoware_install(EXECUTABLES talker listener)
ament_package()
