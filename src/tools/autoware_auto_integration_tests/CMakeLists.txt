cmake_minimum_required(VERSION 3.5)

project(autoware_auto_integration_tests NONE)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "integration_tests-extras.cmake"
)

ament_auto_package(INSTALL_TO_SHARE
  cmake
)
