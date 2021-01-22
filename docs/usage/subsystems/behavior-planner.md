Starting and testing the behavior planner {#behavior-planner-howto}
=======================================

# How to start the stack

Start simulation as described in @ref lgsvl.
Additionally, to configure LGSVL for this demonstration:

1. Maps: use this [map link](https://assets.dev.lgsvlsimulator.com/d5b8bb0b7f49875a8a4bbf83c50b3a4fe53779c7/environment_AutonomouStuff)
2. Vehicles: Select `ROS2 native` bridge type and paste content of `AutowareAuto/lgsvl-sensors.json` into the `Sensors` text box
3. Simulations: In `General` tab, `Select Cluster = Local Machine` and untick any boxes.
In `Map & Vehicles` tab, ensure to untick `Run simulation in interactive mode`.
In `Traffic` tab, untick all selection.
The `Weather` tab is irrelevant

*terminal 1*
```
# start sim according to instructions above but don't drive away yet to make sure we can localize ourselves
```

*terminal 2*
```
> ade enter
ade$ cd AutowareAuto && source install/setup.bash
ade$ colcon build --packages-up-to autoware_auto_avp_demo
ade$ stdbuf -o L ros2 launch autoware_auto_avp_demo ms3_sim.launch.py
```

The `stdbuf` command above is needed because the default in ROS is to only output lines from `stdout` when the buffer is full.
This command changes that setting to use a "line buffer" which outputs every line, providing more debugging information.

*terminal 3*
```
> ade enter
ade$ ros2 topic echo /planning/trajectory
```

Move the vehicle on LGSVL to the position shown in the image.
This is the "pick-up/drop-off zone" on the parking lot roadway in front of the three parking spots directly outside of the front door of the AutonomouStuff building.
![StartPose](images/avp-demo-start-pose.png)

Before selecting a goal, you will need to initialize localization.
To do this switch to the `rviz` window, click the `2D Pose Estimate` button at the top, and then click at the approximate location where the vehicle currently is in the map and drag in the direction of the vehicle's heading.
You can verify that the vehicle has been localized by the vehicle model jumping to the new location and the real-time lider scans matching up with the static lidar map.

Next, to select a parking spot graphically, click the `2D Nav Goal` button in `rviz`, click in the goal location, and drag in the direction of the goal heading.

Optionally, to send a goal position/heading programmatically:

*terminal 4*
```
> ade enter
ade$ ros2 topic pub /planning/goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: -95.875, y: 57.707, z: -1.950}, orientation: {x: -0.021, y: 0.014,z: 0.901,w: 0.434}}'} --once
```

if you want to park in the backward direction, send:
```
> ade enter
ade$ ros2 topic pub /planning/goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: -97.629, y: 59.872, z: 0.0}, orientation: {x: 0.0, y: 0.0,z: -0.43,w: 0.90283}}'} --once
```

## Verify that Behavior Planner receives routes from Global Path
On Terminal 2, you should see following message output:

```
[behavior_planner_node_exe-19] [INFO] [planning.behavior_planner_node]: Received route
```

## Verify that Lane Planner and Parking Planner are called by Behavior Planner
On Terminal 2, you should see the following message output:

```
[behavior_planner_node_exe-19] [INFO] [planning.behavior_planner_node]: sent parking trajectory action goal
[behavior_planner_node_exe-19] [INFO] [planning.behavior_planner_node]: Recieved trajectory from planner
```

## Verify that Behavior Planner outputs a Trajectory for MPC to follow
On Terminal 3, you should see a trajectory message coming out from the behavior planner.
