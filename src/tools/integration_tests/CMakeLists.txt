cmake_minimum_required(VERSION 3.5)

project(apex_integration_tests NONE)

find_package(ament_cmake_core REQUIRED)

ament_package(
  CONFIG_EXTRAS "apex_integration_tests-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
