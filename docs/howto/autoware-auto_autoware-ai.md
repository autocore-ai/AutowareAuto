ROS1 Bridge {#ros1_bridge_title}
============

[TOC]

# ros1_bridge: Bridging AutowareAuto (ROS 2) and AutowareAI (ROS 1) applications {#ros1-bridge}

The [`ros1_bridge`](https://github.com/ros2/ros1_bridge) application provides a network bridge
which enables the exchange of messages between ROS 1 and ROS 2, allowing ROS 1 tools like
`rviz`, `rqt_plot`, `image_view`, and `rosbag` to work with ROS 2 applications.

`ros1_bridge` is limited to only the message/service types available at compile time of the bridge;
hence, you need to compile it from source.

The `ros1_bridge` must be built any time a new message type is
defined.


## 1. Build ros1_bridge {#build-ros1-bridge}

For more details see the
[ROS 2 wiki instructions](https://github.com/ros2/ros1_bridge#building-the-bridge-from-source).

The instructions below assume that you have installed ROS 2 Bouncy (already available in `ade`) and
[ROS 1 Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu). 

If you have defined custom messages in your `~/workspace`, build and source
your workspace before starting (see `ade$ source ~/workspace/install/setup.bash` below).

To clone and compile `ros1_bridge`:

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

To enable `ros1_bridge` by default (for all future terminals), source the install directory in your
`~/.bashrc`. It's also **strongly recommended** to add the following alias, so that you can start
the `ros1_bridge` and `roscore` in one command.

```bash
$ ade enter
ade$ echo "source ~/ros1_bridge_ws/install/setup.bash" >> ~/.bashrc
ade$ echo 'alias start_ros1_bridge="( source /opt/ros/melodic/setup.bash && ( roscore & source ~/ros1_bridge_ws/install/setup.bash && sleep 1 && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics ) && killall roscore ) || killall roscore"' >> ~/.bashrc
ade$ source ~/.bashrc  # To enable it for the current terminal
```

## 2. Test ros1_bridge {#test-ros1-bridge}

To test the `ros1_bridge`, open three terminals:

Terminal 1 - `roscore` and the `ros1_bridge`:

```bash
$ ade enter
ade$ start_ros1_bridge
```

Terminal 2 - Publish a message with ROS 2:
```bash
$ ade enter
ade$ ros2 topic pub /test std_msgs/Bool -r 1
```

Terminal 3 - Listen for the message on ROS 1:
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

If the output in your terminal matches what is shown above, congratulations! Your `ros1_bridge` is
compiled and installed properly.

Enter `Ctrl-C` to stop the applications.

## 3. Run AutowareAuto and AutowareAI applications {#autoware-auto-autoware-ai}
TBD after:
1. https://gitlab.com/AutowareAuto/AutowareAuto/issues/3
2. https://gitlab.com/AutowareAuto/AutowareAuto/issues/4
3. https://gitlab.com/AutowareAuto/AutowareAuto/issues/6