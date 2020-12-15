Autoware.Auto {#index}
======================

# Introduction

[Autoware](https://www.autoware.org/) is the world's first "all-in-one" open-source
software for self-driving vehicles hosted under the Autoware Foundation. Autoware first began with
the [Autoware.AI project](https://www.autoware.ai/), based on
[ROS 1](http://wiki.ros.org/Documentation).

The [Autoware.Auto project](https://www.autoware.auto/), based on [ROS 2](https://index.ros.org/doc/ros2/), is the next generation successor of Autoware.AI. Autoware.Auto is built with modern software engineering best practices including code reviews, continuous integration testing, thorough documentation, thorough test coverage, style and development guides, and a well-defined release process.

Autoware.Auto has two other major differentiators when it's compared to Autoware.AI:

1. Improved system architecture and module interface design (including messages and APIs)
2. An emphasis on reproducibility and determinism at the library, node, and system levels

![Autoware.Auto testing vehicle](images/lexus.jpg)


# Use Cases

Autoware.Auto initially targets the following 2 use cases:

1. Autonomous Valet Parking
2. Autonomous Depot Maneuvering

Uses cases are designed with respect to target Operational Design Domains (ODDs). Demonstrations for Autoware.Auto can be found in \ref demos.

After the initial set of [milestones](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones)
are completed, Autoware.Auto will allow you to easily map a parking lot, create
a map for autonomous driving, and drive over this parking lot entirely autonomously; all in less
than 2 weeks.

# Supported Hardware

1. **Vehicle**: Lexus 450 LH with the
[Pacmod 3.0](https://autonomoustuff.com/product/small-ev-by-wire-kits/) DBW interface
2. **Sensors**:
    1. 4 [VLP-16](https://velodynelidar.com/vlp-16-hi-res.html) (or comparable sensors, e.g.
      VLP-32C)
    2. 16 Sonar sensors
    3. 4 [cameras](http://wiki.ros.org/pointgrey_camera_driver) (180 degree FOV)
    4. [Novatel GPS](https://autonomoustuff.com/product/novatel-vehicle-kits/)
3. **ECUs**:
    1. [Nvidia AGX Xavier](https://www.nvidia.com/en-us/deep-learning-ai/products/agx-systems/) aarch64 computer
    2. [Nuvo](https://autonomoustuff.com/product/astuff-spectra/) rugged x86-64 desktop computer


# Roadmap

Feature planning is being handled via
[GitLab milestones](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones).

# Documentation

Below is the table of contents for the Autoware.Auto documentation.

If you are new to Autoware.Auto, begin at the installation documentation, then move on to the tutorials.

- @subpage installation
- @subpage tutorials
- @subpage demos
- @subpage contributors-guide
- @subpage design


# Coverage report

An automatically-generated code coverage report for the Autoware.Auto codebase is [available here](coverage/index.html).
