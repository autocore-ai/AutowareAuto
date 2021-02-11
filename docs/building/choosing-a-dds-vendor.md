Choosing a DDS Vendor {#choosing-a-dds-vendor}
==============================================

[TOC]

Choosing a DDS vendor is usually as simple as changing the `RMW_IMPLEMENTATION` environment variable. It can be set after building Autoware, as long as the respective `rmw_*` packages are installed before building, because the software is built for all the available typesupports installed. The rmw layer loads a given `rmw_*` implementation at runtime via the `RMW_IMPLEMENTATION` variable, unless there's only one, in which case shortcuts to the installed RMW implementation. The change can also be made more permanent by changing it in the `ADE_DOCKER_RUN_ARGS` in the `.aderc`.

For more information about why you would want to use a different DDS vendor and which ones are available, see [this ROS Index article](https://index.ros.org/doc/ros2/Concepts/About-Different-Middleware-Vendors/).
For more information about working with multiple middleware (DDS) implementations, see [this ROS Index article](https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/).

# For Cyclone DDS (the default in ADE):

```
ade$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
CycloneDDS is required for integration with LGSVL.

# For FastRTPS (now FastDDS - the default in ROS Dashing):
```
ade$ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

# For Connext (not installed to ADE by default):
```
ade$ sudo apt-get install rti-connext-dds-5.3.1 ros-dashing-rmw-connext-cpp
ade$ export RMW_IMPLEMENTATION=rmw_connext_cpp
```

# For GurumDDS (not installed to ADE by default):
```
ade$ sudo apt-get install gurumdds-2.6 ros-dashing-rmw-gurumdds-cpp
ade$ export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
```
