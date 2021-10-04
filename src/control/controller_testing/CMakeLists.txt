cmake_minimum_required(VERSION 3.5)
project(controller_testing)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(autoware_testing REQUIRED)
  add_smoke_test(${PROJECT_NAME} ${PROJECT_NAME}_main.py)
endif()

# Install python scripts
ament_python_install_package(${PROJECT_NAME})

# Install the main_script file where ros2 is looking for it
set(PY_EXECUTEABLE "controller_testing_main.py")
install(PROGRAMS ${PROJECT_NAME}/${PY_EXECUTEABLE} DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  param
)
