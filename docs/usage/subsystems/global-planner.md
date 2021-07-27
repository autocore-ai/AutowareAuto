Starting and testing the global planner {#global-planner-howto}
=======================================

@tableofcontents

# How to start the stack

Start simulation as described in @ref lgsvl.
Additionally, to configure LGSVL for this demonstration:

1. Maps: use this [map link](https://assets.dev.lgsvlsimulator.com/d5b8bb0b7f49875a8a4bbf83c50b3a4fe53779c7/environment_AutonomouStuff)
2. Vehicles: Select `ROS2 native` bridge type and paste the content of `AutowareAuto/src/tools/autoware_demos/config/svl/avp-sensors.json` into the `Sensors` text box
3. Simulations: In `General` tab, `Select Cluster = Local Machine` and untick any boxes.
In `Map & Vehicles` tab, ensure to untick `Run simulation in interactive mode`.
In `Traffic` tab, untick all selection.
The `Weather` tab is irrelevant.

*terminal 1*

```{bash}
# start sim according to instructions above but don't drive away yet to make sure we can localize ourselves
# then start RViz2 for visualization
> ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_auto_launch autoware_auto_visualization.launch.py
```

*terminal 2*
```{bash}
> ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ stdbuf -o L ros2 launch autoware_demos avp_sim.launch.py
```

The `stdbuf` command above is needed because the default in ROS is to only output lines from `stdout` when the buffer is full.
This command changes that setting to use a "line buffer" which outputs every line, providing more debugging information.

*terminal 3*
```{bash}
> ade enter
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 topic echo /planning/global_path
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
```{bash}
> ade enter
ade$ ros2 topic pub --once /planning/goal_pose geometry_msgs/msg/PoseStamped "{
header:
  {stamp:
    {sec: 1600775035,
    nanosec: 496432027},
  frame_id: map},
pose:
  {position:
    {x: -77.87589263916016,
    y: -18.580652236938477,
    z: 0.0},
  orientation:
    {x: 0.0,
    y: 0.0,
    z: 0.14149930538744868,
    w: 0.9899383549367453}}}
"
```

The path with lane IDs should be output in *terminal 3*.

**Note** To choose a different parking spot, click `2D Nav Goal` in `rviz` and listen to the message with

    ros2 topic echo /goal_pose

and update the values in the message in YAML format above.

# Passing metrics

The output message looks like

```{yaml}
...
primitives:
- id: 8252
  primitive_type: parking
- id: 9074
  primitive_type: drivable_area
- id: 6581
  primitive_type: lane

# lots of lanes omitted

- id: 6700
  primitive_type: lane
- id: 7957
  primitive_type: drivable_area
- id: 9824
  primitive_type: parking
```

To check if the route is reasonable, open the OSM map `AutowareAuto/src/tools/autoware_demos/data/autonomousstuff_parking_lot.osm` in a text editor and search for `way id='9824`.
It references a node, the center of the entrance line, `9831` in this case.
Searching for its coordinates, they are:
```{xml}
<node id='9831' visible='true' version='1' lat='37.38065126054' lon='-121.90931385745'>
```
To graphically inspect this, install the `qgis` tool with:
```{bash}
sudo apt install qgis
```
and follow [this tutorial](https://wiki.openstreetmap.org/wiki/QGIS_tutorial) to open the .osm file.
Finally, install a plugin called `Lat Lon Tools` and enter the coordinates to pinpoint the node with a red crosshair.
