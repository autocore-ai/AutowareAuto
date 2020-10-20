cmake_minimum_required(VERSION 3.5)
project(motion_model_testing_simulator)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install python scripts
ament_python_install_package(${PROJECT_NAME})

ament_auto_package()
