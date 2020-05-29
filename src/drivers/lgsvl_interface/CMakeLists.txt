cmake_minimum_required(VERSION 3.5)
project(lgsvl_interface)
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
ament_auto_find_build_dependencies()

# Generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lgsvl_interface.cpp
  src/lgsvl_interface_node.cpp)
autoware_set_compile_options(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-useless-cast -Wno-double-promotion -Wno-old-style-cast -Wno-sign-conversion)

rclcpp_components_register_nodes(
  ${PROJECT_NAME}
  "lgsvl_interface::LgsvlInterfaceNode")

ament_auto_add_executable(${PROJECT_NAME}_exe
  src/lgsvl_interface_main.cpp)
autoware_set_compile_options(${PROJECT_NAME}_exe)
target_link_libraries(${PROJECT_NAME}_exe ${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  # gtest
  set(LGSVL_INTERFACE_GTEST test_${PROJECT_NAME}_gtest)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${LGSVL_INTERFACE_GTEST}
    test/src/gtest_main.cpp
    test/src/test_lgsvl_interface.cpp)
  target_include_directories(${LGSVL_INTERFACE_GTEST} PRIVATE "include" "test/include")
  target_link_libraries(${LGSVL_INTERFACE_GTEST} ${PROJECT_NAME})
  add_dependencies(${LGSVL_INTERFACE_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${LGSVL_INTERFACE_GTEST}
    "autoware_auto_common"
    "autoware_auto_msgs")
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    param
    launch
)
