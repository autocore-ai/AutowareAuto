LGSVL Interface
===============


This package is a vehicle interface compatible with the LGSVL simulator.

This document outlines minimal setup and usage instructions for the simulator with the interface.

# Setup

The following components are required:

1. [ADE](https://ade-cli.readthedocs.io/en/latest/install.html) with:
    1. ROS Melodic
    2. Autoware.Auto
2. [LGSVL Simulator](https://github.com/lgsvl/simulator/releases/tag/2019.10)
3. [ros1_bridge](https://github.com/ros2/ros1_bridge)
4. [rosbridge_suite](http://wiki.ros.org/rosbridge_suite/Tutorials/RunningRosbridge)
5. [ROS 1 Autoware messages](https://gitlab.com/autowarefoundation/autoware.ai/messages)

## Installation

1. To install ade, follow
the [installation instructions](https://ade-cli.readthedocs.io/en/latest/install.html)
2. To obtain the Autoware.Auto and ROS Melodic image, start `ade` with the `.aderc` file
provided:
```
$ ade start -- --privileged -p 127.0.0.1:9090:9090
```
Where `--privileged` is provided to add joystick support, and the latter argument is to mount the
ports for the ROS web bridge
3. To install LGSVL on the host system, download the appropriate zip file from the
[2019.10 release](https://github.com/lgsvl/simulator/releases/tag/2019.10). It is assumed that this
zip file will be extracted to a folder called `lgsvl` in the root workspace on the host
4. To install `rosbridge_suite`, run:
```
$ ade enter
ade$ sudo apt update
ade$ sudo apt install ros-melodic-rosbridge-suite
```
5. To obtain the source code for the messages, run:
```
$ ade enter
ade$ mkdir -p ros1_ws/src
ade$ cd ros1_ws/src
ade$ git clone https://gitlab.com/autowarefoundation/autoware.ai/messages --branch 1.13.0
```
6. To obtain the source code for `ros1_bridge`, run:
```
$ ade enter
ade$ mkdir -p ros1_bridge_ws/src
ade$ cd ros1_bridge_ws/src
ade$ git clone https://github.com/ros2/ros1_bridge --branch dashing
```

## Building

To build the messages:

```
$ ade enter
ade$ source /opt/ros/melodic/setup.bash
ade$ cd ~/ros1_ws
ade$ catkin_make
```

To build `ros1_bridge` with custom message support:

```
$ ade enter
ade$ source /opt/ros/melodic/setup.bash
ade$ source ~/ros1_ws/devel/setup.bash
ade$ source /opt/AutowareAuto/setup.bash
ade$ cd ~/ros1_bridge_ws
ade$ colcon build
```

# Running

To run the minimal joystick control demo, do the following:

1. Start the simulator on the host:
```
$ cd ~/lgsvl
$ ./simulator
```
    1. Ensure the chosen vehicle is set up with the `ROS` bridge type
    2. Ensure the sensor configuration contains the
    [Vehicle Control](https://www.lgsvlsimulator.com/docs/sensor-json-options/#vehicle-control)
    sensor; add the following to the vehicle configuration json file:
    ```json
    {
      "type": "Vehicle Control",
      "name": "AD Car Control",
      "params":
      {
        "Topic": "/vehicle_cmd"
      }
    }
    ```
    3. Ensure the simulator listens on the same websocket as the `rosbridge_suite` (default 9090)
    4. Run the simulation
2. Start `roscore` (to support `ros1_bridge` and `rosbridge_suite`):
```
$ ade enter
ade$ source /opt/ros/melodic/setup.bash
ade$ roscore
```
3. Start `ros1_bridge`:
```
$ ade enter
ade$ export APEX_RMW_DISABLE_INTRA=1
ade$ source ros1_bridge_ws/install/setup.bash
ade$ ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```
4. Start `rosbridge_suite`:
```
$ ade enter
ade$ source /opt/ros/melodic/setup.bash
ade$ source ros1_ws/devel/setup.bash
ade$ roslaunch rosbridge_server rosbridge_websocket.launch
```
5. Start the vehicle interface and joystick controller:
```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch lgsvl_interface lgsvl_joystick.launch.py
```

By default, the right trigger controls throttle, the left trigger control braking, and the left
joystick controls the steer angle.

# Troubleshooting

## There are errors while building ros1_bridge

The following warnings are expected during the build process:

```
--- stderr: ros1_bridge
3 mappings can not be generated due to missing dependencies:
- autoware_msgs/Lane <-> autoware_msgs/Lane:
  - autoware_msgs/Waypoint
- autoware_msgs/LaneArray <-> autoware_msgs/LaneArray:
  - autoware_msgs/Lane
- autoware_msgs/Waypoint <-> autoware_msgs/Waypoint:
  - autoware_msgs/WaypointState

---
Finished <<< ros1_bridge [3min 16s]

Summary: 1 package finished [3min 16s]
1 package had stderr output: ros1_bridge

```

This warning occurs because ROS 2 message definitions are not present for the listed types. This
does not affect functionality of the vehicle interface as it does not use these types.

If the package fails to build, try different permutations of sourcing workspaces, or contact
customer support.

## The vehicle handles strangely

This is a limitation of the current interface used with LGSVL.

LGSVL currently obeys all velocity command inputs as being verbatim, without taking into account
vehicle dynamics. As such, there is some vastly simplified vehicle dynamics logic in the vehicle
interface.

## The brake/throttle/steering does not work

The joystick control mapping is not deterministic. It is occasionally necessary to modify the axis
mapping.

First, with the joystick controller running, verify that you can see the raw messages by running
the following:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic echo /joy
```

Next, actuate the appropriate axis on the vehicle controllers

## There is no data on the /joy topic

Ensure that `/dev/input/js0` is available from within ade.

If it is not available, restart `ade`, ensuring that it is run with the `--privileged` flag.

## The vehicle still does not move

First, ensure the whole stack is running properly, and is appropriately configured.

Next, ensure there is data on the `/joy` topic. If this is not the case, refer to the appropriate
question.

Next, check that data arrives on each of the following topics:

```
$ ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic hz /joy
ade$ ros2 topic hz /raw_command
ade$ ros2 topic hz /vehicle_cmd
```

If data is not arriving on one of these topics, then the stack did not start up correctly, or there
is a configuration problem. Stop the vehicle interface and joystick controller and inspect the logs
to determine which is the case.

Next, check that data is arriving on the ROS 1 side:

```
$ ade enter
ade$ source /opt/ros/melodic/setup.bash
ade$ source ros1_ws/devel/setup.bash
ade$ rostopic hz /vehicle_cmd
```

If data does not arrive, then there may be an issue with the building or running of ros1_bridge.

If data does arrive, and the vehicle is still not moving, ensure that the port (9090 by default)
needed by the web bridge is exposed by running the following command:

```
$ ss -lnt
```

If the port is not exposed, make sure ade exposes the port on local loopback by restarting ADE with
the added arguments:

```
$ ade start <ade arguments> -- <other docker arguments> -p 127.0.0.1:<PORT>:<PORT>
```

If all of the above work, and the issues still persist, contact customer support.
