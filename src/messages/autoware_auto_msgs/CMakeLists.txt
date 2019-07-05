# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(autoware_auto_msgs)

# Generate messages
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(autoware_auto_msgs
  # Helper types
  "msg/Complex32.msg"
  "msg/DiagnosticHeader.msg"
  "msg/TrajectoryPoint.msg"
  # Interfaces
  "msg/ControlDiagnostic.msg"
  "msg/Trajectory.msg"
  "msg/VehicleControlCommand.msg"
  "msg/VehicleKinematicState.msg"
  "msg/VehicleOdometry.msg"
  "msg/VehicleStateCommand.msg"
  "msg/VehicleStateReport.msg"
  # Implementation-specific messages
  "msg/PointClusters.msg"
  DEPENDENCIES "std_msgs" "geometry_msgs" "sensor_msgs")

ament_export_dependencies("std_msgs" "geometry_msgs" "sensor_msgs")

ament_package()

