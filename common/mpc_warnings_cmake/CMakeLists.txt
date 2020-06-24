# ament wrapper for ACADO toolkit
cmake_minimum_required(VERSION 3.5)

project(mpc_warnings_cmake)

find_package(ament_cmake REQUIRED)

# Install cmake helper functions
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})

ament_package(CONFIG_EXTRAS "mpc_warnings_cmake-extras.cmake")
