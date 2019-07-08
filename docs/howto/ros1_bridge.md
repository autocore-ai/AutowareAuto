ROS 1 bridge {#ros1_bridge_title}
============

[TOC]

# ros1_bridge: bridging Autoware.Auto (ROS 2) and Autoware.AI (ROS 1) applications {#ros1-bridge}

The [ros1_bridge](https://github.com/ros2/ros1_bridge) application provides a network bridge
to enable the exchange of messages between ROS 1 and ROS 2, allowing ROS 1 tools like
`rviz`, `rqt_plot`, `image_view`, and `rosbag` to work with ROS 2 applications.

This article explains how to build the `ros1_bridge` within ADE, and validate the connection between
Autoware.Auto and Autoware.AI using a `pcap` file, the
[velodyne_node](https://gitlab.com/AutowareAuto/AutowareAuto/tree/master/src/drivers/velodyne_node),
the `ros1_bridge`, and `rviz`.


## Requirements

The `ros1_bridge` is limited to the message/service types that are available at compile time of the
bridge; therefore, the `ros1_bridge` must be compiled from source. The `ros1_bridge` **must be built
when a new message type is defined**.

The instructions below assume that ROS 2 Dashing and ROS 1 Melodic have been installed (both already
available in `ade`).

If custom messages have been defined in the `~/workspace`, build and source the workspace containing
the message definitions before starting (see `ade$ source ~/workspace/install/setup.bash` below).


## 1. Build the ros1_bridge {#build-ros1-bridge}

For more details, see the
[ROS 2 wiki instructions](https://github.com/ros2/ros1_bridge#building-the-bridge-from-source).

Follow the steps below to clone and compile the `ros1_bridge`:

```bash
$ ade enter
ade$ source ~/workspace/install/setup.bash  # Only if you have custom messages
ade$ mkdir -p ~/ros1_bridge_ws/src
ade$ cd ~/ros1_bridge_ws/src
ade$ git clone https://github.com/ros2/ros1_bridge.git --branch 0.7.2
ade$ cd ..
ade$ source /opt/ros/melodic/setup.bash
ade$ colcon build --merge-install --packages-select ros1_bridge
```

\note
After sourcing ROS 1 Melodic, the following warning will appear; it can safely be ignored while
following this workflow:
```
ade$ source /opt/ros/melodic/setup.bash
ROS_DISTRO was set to 'dashing' before. Please make sure that the environment does not mix paths from
different distributions.
```

To enable the `ros1_bridge` by default (for all future terminals), source the install directory in
the `~/.bashrc` file. It's also **strongly recommended** to add the following alias so that the
`ros1_bridge` and `roscore` can be started with a single command.

```bash
$ ade enter
ade$ echo "source ~/ros1_bridge_ws/install/setup.bash" >> ~/.bashrc
ade$ echo 'alias start_ros1_bridge="( source /opt/ros/melodic/setup.bash && ( roscore & source ~/ros1_bridge_ws/install/setup.bash && sleep 1 && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics ) && killall roscore ) || killall roscore"' >> ~/.bashrc
ade$ source ~/.bashrc  # To enable it for the current terminal
```


## 2. Test the ros1_bridge {#test-ros1-bridge}

To perform a simple test of the `ros1_bridge`, open three terminals:

ADE Terminal 1 - start `roscore` and `ros1_bridge`:

```bash
$ ade enter
ade$ start_ros1_bridge
```

ADE Terminal 2 - publish a message with ROS 2:

```bash
$ ade enter
ade$ ros2 topic pub /test std_msgs/Bool -r 1
```

ADE Terminal 3 - listen for the message on ROS 1:

```bash
$ ade enter
ade$ source /opt/ros/melodic/setup.bash
ade$ rostopic echo /test
data: False
---
data: False
---
...
```

If the output in the terminal matches what is shown above, congratulations! The `ros1_bridge` is
compiled and installed properly.

Enter `Ctrl-C` to stop the applications.


## 3. Run Autoware.Auto and Autoware.AI applications {#autoware-auto-autoware-ai}

For this section, we will follow the instructions in the [3D Perception stack](@ref perception-stack) howto, but
we will run `rviz` instead of `rviz2` to demonstrate that ROS 1 applications, such as `rviz`, can
interact with Autoware.Auto.

We will need a new terminal, where we will start `roscore` and `ros1_bridge`:

```bash
$ ade enter
ade$ start_ros1_bridge
```

In the terminal where were going to start `rviz2` (when following the [3D Perception stack](@ref perception-stack) howto), we will instead run the following:

```bash
$ ade enter
ade$ export LD_LIBRARY_PATH=/usr/local/nvidia/lib64/  # see the note below
ade$ source /opt/ros/melodic/setup.bash
ade$ rviz -d /home/"${USER}"/AutowareAuto/install/share/autoware_examples/rviz/autoware.rviz
```

After this, we should be presented with a screen similar to this:

![Autoware.Auto velodyne_node point cloud snapshot](autoware-auto-velodyne-node-point-cloud-snapshot.png)

The ROS 1 `rostopic` tool can also be used to validate the
flow of sensor data from the pcap file, through the `velodyne_node`, and through the `ros1_bridge`
to the ROS 1 domain:

```bash
$ ade enter
ade$ source /opt/ros/melodic/setup.bash
ade$ rostopic echo /test_velodyne_node_cloud_front
```
