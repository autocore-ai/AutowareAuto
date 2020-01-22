# All rights reserved.
cmake_minimum_required(VERSION 3.5)
project(autoware_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# Generate messages
rosidl_generate_interfaces(lgsvl_interface_msgs
  "msg/AccelCmd.msg"
  "msg/BrakeCmd.msg"
  "msg/ControlCommand.msg"
  "msg/LampCmd.msg"
  "msg/SteerCmd.msg"
  "msg/VehicleCmd.msg"
  DEPENDENCIES "std_msgs")

ament_auto_package()
