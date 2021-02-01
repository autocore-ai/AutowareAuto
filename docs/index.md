Autoware.Auto {#index}
======================

# Introduction

[Autoware](https://www.autoware.org/) is the world's first "all-in-one" open-source software for self-driving vehicles hosted under the Autoware Foundation.

The [Autoware.Auto project](https://www.autoware.auto/), based on [ROS 2](https://index.ros.org/doc/ros2/), is the next generation successor of the [Autoware.AI project](https://www.autoware.ai/), based on [ROS 1](http://wiki.ros.org/Documentation).

The major differentiators of Autoware.Auto compared to Autoware.AI are:

1. Modern software engineering best practices including code reviews, continuous integration testing, thorough documentation, thorough [test coverage](coverage/index.html), style and development guides
2. Improved system architecture and module interface design (including messages and APIs)
3. An emphasis on reproducibility and determinism at the library, node, and system levels


# Functionality

Autoware.Auto initially targets the following two use cases:

1. Autonomous Valet Parking
2. Autonomous Cargo Delivery

As of January 2021, the valet parking functionality has been implemented and was showcased in a [live vehicle demonstration](https://www.youtube.com/watch?v=MC7n8vwiLcg). It allows maneuvering at low speeds on a parking lot.

The cargo delivery functionality is in development. It will allow driving in a more difficult environment with intersections and indoor navigation, at moderate speeds.
Each use case is designed with respect to a target Operational Design Domain (ODD), and the current milestones for development can be viewed [here](https://gitlab.com/autowarefoundation/autoware.auto/-/milestones).


# Getting started

* First, install Autoware.Auto: @subpage installation.
* After that, continue with @subpage usage.

# Contributing
* @subpage building has instructions and troubleshooting for compiling Autoware.Auto.
* To get an overview of the algorithms and individual packages in Autoware.Auto, read @subpage design.
* As you start working on the source code or documentation, please see the @subpage contributors-guide.

![Autoware.Auto testing vehicle](images/lexus.jpg)
