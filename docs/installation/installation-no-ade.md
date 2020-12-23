Installation w/o ADE {#installation-no-ade}
====================

[TOC]

# Goals {#installation-noade-goals-noade}


This article demonstrates how to successfully build [Autoware.Auto](https://www.autoware.auto/) applications without the ade framework.


# Installation Requirements {#installation-noade-install-requirements}

To compile [Autoware.Auto project](https://www.autoware.auto/) from sources, the following tools must be installed in the system.

- Apt packages
```bash
$ sudo apt install -y git cmake python3-pip
```
- Python modules
```bash
$ pip3 install -U colcon-common-extensions vcstool
```

# ROS 2 core {#installation-noade-ros2-core}

First, the [ROS 2](https://index.ros.org/doc/ros2/) core components and tools must be installed. The full guide is available at [ROS 2 Installation](https://index.ros.org/doc/ros2/Installation/).
Once installed source the setup file.

```bash
source /opt/ros/<distro>/setup.bash
```
# ROS 2 package dependencies {#installation-noade-ros2-dependencies}

[Autoware.Auto project](https://www.autoware.auto/) requires some [ROS 2](https://index.ros.org/doc/ros2/) packages in addition to the core components.
The tool `rosdep` allows an automatic search and installation of such dependencies.

```bash
$ sudo apt update
$ apt install -y python3-rosdep
$ rosdep init
$ rosdep update
```

Once installed, dependencies can be deduced from the sources of the [Autoware.Auto project](https://www.autoware.auto/).

```bash
$ git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
$ cd AutowareAuto
$ vcs import < autoware.auto.$ROS_DISTRO.repos
$ rosdep install -y -i --from-paths src
```

# Choosing a DDS Vendor

Choosing a DDS vendor is usually as simple as changing the `RMW_IMPLEMENTATION` environment variable.
This variable can be changed at runtime, but setting it before compiling ensures the correct
libraries are built against.
For more information about why you would want to use a different DDS vendor and which ones are available, see [this ROS Index article](https://index.ros.org/doc/ros2/Concepts/DDS-and-ROS-middleware-implementations/).
For more information about working with multiple middleware (DDS) implementations, see [this ROS Index article](https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/).

## For Cyclone DDS (the default in `ade`):

```
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## For FastRTPS (now FastDDS - the default in ROS Dashing):
```
$ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## For Connext (not installed to `ade` by default):
```
$ sudo apt-get install rti-connext-dds-5.3.1 ros-$ROS_DISTRO-rmw-connext-cpp
$ export RMW_IMPLEMENTATION=rmw_connext_cpp
```

## For GurumDDS (not installed to `ade` by default):
```
$ sudo apt-get install gurumdds-2.6 ros-$ROS_DISTRO-rmw-gurumdds-cpp
$ export RMW_IMPLEMENTATION=rmw_gurumdds_cpp
```

# How to build {#installation-noade-how-to-build}

```bash
$ cd AutowareAuto
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
$ colcon test
$ colcon test-result
```
