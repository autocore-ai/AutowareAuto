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
$ sudo apt install -y python3-rosdep
$ sudo rosdep init
$ rosdep update
```

Once installed, dependencies can be deduced from the sources of the [Autoware.Auto project](https://www.autoware.auto/).

```bash
$ git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
$ cd AutowareAuto
$ vcs import < autoware.auto.$ROS_DISTRO.repos
$ rosdep install -y -i --from-paths src
```

Next, to compile the source code, see @ref building.