ROS 1 bridge {#ros1_bridge_title}
============

[TOC]

# ros1_bridge: bridging Autoware.Auto (ROS 2) and Autoware.AI (ROS 1) applications {#ros1-bridge}

The [ros1_bridge](https://github.com/ros2/ros1_bridge) application provides a network bridge
to enable the exchange of messages between ROS 1 and ROS 2, allowing ROS 1 tools like
`rviz`, `rqt_plot`, `image_view`, and `rosbag` to work with ROS 2 applications.

The `ros1_bridge` is limited to the message/service types that are available at compile time of the
bridge; therefore, the `ros1_bridge` must be compiled from source.

The `ros1_bridge` must be built when a new message type is defined.


## 1. Build the ros1_bridge {#build-ros1-bridge}

For more details, see the
[ROS 2 wiki instructions](https://github.com/ros2/ros1_bridge#building-the-bridge-from-source).

The instructions below assume that ROS 2 Bouncy has been installed (already available in `ade`), as
well as [ROS 1 Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).

If custom messages have been defined in the `~/workspace`, build and source the workspace containing
the message definitions before starting (see `ade$ source ~/workspace/install/setup.bash` below).

Follow the steps below to clone and compile the `ros1_bridge`:

```bash
$ ade enter
ade$ source ~/workspace/install/setup.bash  # Only if you have custom messages
ade$ mkdir -p ~/ros1_bridge_ws/src
ade$ cd ~/ros1_bridge_ws/src
ade$ git clone https://github.com/ros2/ros1_bridge.git --branch 0.5.0
ade$ cd ..
ade$ source /opt/ros/melodic/setup.bash
ade$ colcon build --merge-install --packages-select ros1_bridge
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

To test the `ros1_bridge`, open three terminals:

Terminal 1 - start `roscore` and `ros1_bridge`:

```bash
$ ade enter
ade$ start_ros1_bridge
```

Terminal 2 - publish a message with ROS 2:

```bash
$ ade enter
ade$ ros2 topic pub /test std_msgs/Bool -r 1
```

Terminal 3 - listen for the message on ROS 1:

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
TBD after:

1. https://gitlab.com/AutowareAuto/AutowareAuto/issues/3
2. https://gitlab.com/AutowareAuto/AutowareAuto/issues/4
3. https://gitlab.com/AutowareAuto/AutowareAuto/issues/6
