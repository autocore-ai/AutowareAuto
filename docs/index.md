Autoware.Auto {#index}
======================

# Autoware.Auto

[Autoware](https://www.autoware.org/) is the world's first "all-in-one" open-source
software for self-driving vehicles hosted under the Autoware Foundation. Based on
[ROS 2](https://index.ros.org/doc/ros2/).

Autoware.Auto as one of the projects of Autoware Foundation is a clean slate rewrite
of [Autoware.AI](https://autoware.ai/). Compared to Autoware.AI, Autoware.Auto has
best possible software engineering practices which includes PR reviews, PR builds,
100% documentation, 100% code coverage, style guide, development and release process, …

It also does two more things differently: a) we have crisply defined interfaces for
different modules (messages and APIs) and  b) architecture designed for determinism
such that it is possible to reproduce behaviors on live and development machines.

![Autoware.Auto testing vehicle](images/lexus.jpg)

# Use Cases
Autoware.Auto will initially address the following 2 use case:
1. Autonomous Valet Parking
1. Autonomous Depot Maneuvering

After the initial set of [milestones](https://gitlab.com/AutowareAuto/AutowareAuto/milestones)
will be completed, Autoware.Auto will allow you to easily map a parking lot, create
a map for autonomous driving and drive over this parking lot entirely autonomously
in less than 2 weeks.

# Supported Hardware
**Car:** Lexus 450 LH with [Pacmod 3.0](https://autonomoustuff.com/product/small-ev-by-wire-kits/)
DBW interface.

**Sensors:**
    1. 4 [VLP16](https://velodynelidar.com/vlp-16-hi-res.html) (or comparable sensors, e.g. VLP-32C)
    2. 16 Sonar sensors
    3. 4 [cameras](http://wiki.ros.org/pointgrey_camera_driver) (180 FOV)
    4. [Novatel GPS](https://autonomoustuff.com/product/novatel-vehicle-kits/)

**ECUs:**
	1. [Nvidia AGX Xavier](https://www.nvidia.com/en-us/deep-learning-ai/products/agx-systems/) aarch64 computer
	1. [Nuvo](https://autonomoustuff.com/product/astuff-spectra/) rugged x86-64 desktop computer

# Documentation

The latest documentation corresponding to the ``master`` branch can be found here:
https://autowareauto.gitlab.io/AutowareAuto/.


# Installation and development

Install AutowareAuto and learn how to develop applications.

- @subpage installation-and-development


# Links to other resources

- @subpage howto
- @subpage tutorials
- @subpage design "Design documents"
- @subpage coverage "Coverage reports"
