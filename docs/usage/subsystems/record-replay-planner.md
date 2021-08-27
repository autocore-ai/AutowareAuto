Record/Replay Planner {#recordreplay-planner-howto}
=====================

[TOC]

# Record/Replay a Trajectory in LGSVL

Autoware.Auto is capable of recording a path of waypoints to disk and then loading and attempting to follow that path.
This demonstration combines several subsystems including @ref ndt-initialization "NDT Localization" and the @ref perception-stack-howto.
To test this functionality, do the following:

## Prerequisites

The following instructions assume you are running the demo inside of ADE.
For instructions on setting up an ADE environment, see @ref installation-ade.
For instructions on setting up Autoware.Auto without ADE, see @ref installation-no-ade.
If running outside of ADE, replace `source /opt/AutowareAuto/setup.bash` with `source /<path_to_your_autoware_folder>/install/setup.bash`.

## Instructions

- [Launch the LGSVL simulator](@ref lgsvl) with the Lexus RX 450h and the AutonomouStuff Parking Lot map. Do not start the simulation at this point.
- In a new terminal:
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_demos recordreplay_planner_demo.launch.py
```

In `rviz`, locate the spawn point of the vehicle in the AutonomouStuff parking lot map using the point cloud as a reference.

@image html images/recordreplay_spawn_point.png "LGSVL Spawn Point" width=800px

Use the "2D Pose Estimate" tool in `rviz` to provide an initial pose estimate for localization.

@image html images/recordreplay_pose_estimate.jpeg "Initial Pose Estimate" width=800px

Using the LGSVL web interface, start the simulation.
Once localization has begun estimating the vehicle's position, the view in `rviz` will jump away from the vehicle. Re-center the view on the vehicle.
In a new terminal:

```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 action send_goal /planning/recordtrajectory autoware_auto_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```

In LGSVL, drive the vehicle around to record the path.
When finished recording, go to the terminal in which you ran the `ros2 action send_goal` command and hit `CTRL+C` to stop recording.  
In LGSVL, hit F12 to re-center the vehicle at the default spawn point.  
Stop the Autoware Stack by pressing `CTRL+C` in terminal where `ros2 launch autoware_auto_avp_demo recordreplay_planner_demo.launch.py` is running.  
Start the Autoware stack by running `ros2 launch autoware_auto_avp_demo recordreplay_planner_demo.launch.py`.  
You will then need to re-initialize localization at the new location with the 2D Pose Estimate tool in `rviz` or by publishing the predefined pose as given in this [link](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/ndt-initialization.html).  

To replay the recorded path:

```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 action send_goal /planning/replaytrajectory autoware_auto_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```

When the vehicle reaches to the goal of the replayed trajectory, the planner stops automatically and outputs `status::SUCCEED` to the terminal in which you ran the `ros2 action send_goal` command.
You can modify end conditions by tuning parameters in [recordreplay_planner.param.yaml](src/tools/autoware_demos/param/recordreplay_planner.param.yaml). The planner terminates planning when both of the following conditions are satisfied:
* goal_distance_threshold_m: threshold for the distance between `nav_base` frame and the last point in the replayed trajectory
* goal_angle_threshold_rad: threshold for the heading angle between `nav_base` frame and the last point in the replayed trajectory

### Controller option

By default the above launch command will run the planner with the MPC controller. You can run the pure pursuit controller instead of the MPC controller by running the following command,
```{bash}
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 launch autoware_demos recordreplay_planner_demo.launch.py run_pure_pursuit:=True
```

Note that the pure pursuit controller is more primitive and does not stop for objects on the path that could cause collision
