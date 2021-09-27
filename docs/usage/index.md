Usage {#usage}
=====

# Preparation
The instructions linked below assume that you have "sourced your installation".
First, you must be in an ADE and/or have built Autoware.Auto yourself.

Then, execute either of those two:
```{bash}
# To use the preinstalled Autoware.Auto in ADE:
ade$ source /opt/AutowareAuto/setup.bash
# To use the Autoware you built yourself:
source ~/AutowareAuto/install/setup.bash
```

That will set a few environment variables, e.g. `$AMENT_PREFIX_PATH`. To reset those variables, it's easiest to just open a new terminal.

If you forget to source the installation, trying to run any `ros2 run` or `ros2 launch` commands will only print something like:

```{bash}
Package 'autoware_demos' not found: "package 'autoware_demos' not found, searching: ['/opt/ros/foxy']"
```

Many launch configurations rely on a point cloud map, which is managed via `git lfs`. To download it, do
```{bash}
git lfs pull --exclude="" --include="*"
```

In case you run multiple ROS2 applications on machines in the same network, they will interfere with each other.
To avoid it, set the environment variable `ROS_DOMAIN_ID` to a distinct value between 0 and 232 on the machines.

# Operational Design Domain (ODD) Demos {#usage-odd-demos}

Autoware.Auto includes a growing number of demonstrations for target Operational Design Domains (ODDs), where an ODD is a formal definition of the set of conditions and circumstances an automated vehicle is designed to operate under (for reference, see [SAE J3016 section 3.22](https://www.sae.org/standards/content/j3016_201806/)).

Each demonstration exhibits the capabilities of the software within the ODD, such as the ability to park a car in a parking lot ODD, or the ability to drive a route through an urban setting in an urban ODD.
Most demonstrations are intended to be used on both a real car and in a simulator.
Each article below contains instructions on how to set up, launch and control one demonstration.
For demonstrations that work on a real car, the hardware requirements are also described.

- @subpage avpdemo (@subpage avpdemo-impressions "Impressions")

# General Demos {#usage-general-demos}

The following pages describe how to run demonstrations which showcase specific pieces of functionality within the Autoware.Auto architecture.

- @subpage behavior-planner-howto
- @subpage global-planner-howto
- @subpage ndt-initialization
- @subpage ekf-localization-howto
- @subpage perception-stack-howto
- @subpage recordreplay-planner-howto
- @subpage rosbag-localization-howto
- @subpage scenario-simulator-howto
