cmake_minimum_required(VERSION 3.5)
project(lgsvl_interface)
set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_common REQUIRED)
ament_auto_find_build_dependencies()

# Generate library
ament_auto_add_library(lgsvl_interface
  src/lgsvl_interface.cpp
  src/lgsvl_interface_node.cpp)
autoware_set_compile_options(lgsvl_interface)
target_compile_options(lgsvl_interface PRIVATE
  -Wno-useless-cast -Wno-double-promotion -Wno-old-style-cast -Wno-sign-conversion)

# Generate executalbe
ament_auto_add_executable(lgsvl_interface_exe src/lgsvl_interface_main.cpp)
autoware_set_compile_options(lgsvl_interface)
target_compile_options(lgsvl_interface PRIVATE -Wno-useless-cast -Wno-double-promotion) # tf2...

if(BUILD_TESTING)
  autoware_static_code_analysis()
endif()

ament_auto_package(INSTALL_TO_SHARE param/ launch/)
