cmake_minimum_required(VERSION 3.5)
project(lgsvl_interface)
set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
ament_auto_find_build_dependencies()

# Generate library
ament_auto_add_library(${PROJECT_NAME}
  src/lgsvl_interface.cpp
  src/lgsvl_interface_node.cpp)
autoware_set_compile_options(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wno-useless-cast -Wno-double-promotion -Wno-old-style-cast -Wno-sign-conversion)

# Generate executalbe
ament_auto_add_executable(lgsvl_interface_exe src/lgsvl_interface_main.cpp)
autoware_set_compile_options(${PROJECT_NAME})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-useless-cast -Wno-double-promotion) # tf2...

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  # gtest
  set(LGSVL_INTERFACE_GTEST test_${PROJECT_NAME}_gtest)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${LGSVL_INTERFACE_GTEST}
    test/src/gtest_main.cpp
    test/src/test_lgsvl_interface.cpp)
  target_include_directories(${LGSVL_INTERFACE_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${LGSVL_INTERFACE_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${LGSVL_INTERFACE_GTEST} "autoware_auto_msgs")
endif()

ament_auto_package(INSTALL_TO_SHARE param/ launch/)
