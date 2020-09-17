Parking planner nodes {#parking-planner-nodes}
==========================

# Purpose / Use cases

This package provides a ROS2 node for the computational package `parking_planner`. 
This is done to separate the computation part from the ROS2-specific part. 
See the computational part for a rationale of what the functionality is to be used for.


# Design

This is a wrapper around `parking_planner`. 
Its behavior can be controlled via single action, which is to plan a trajectory.


## Assumptions / Known limits

See the `parking_planner` design documentation for more details.


## Inputs / Outputs / API

Actions:

This node uses an action to control its behavior:

* `PlanParkingManeuver.action` is used to plan a parking maneuver. 
  It takes a while, and while it is running, it currently does not publish any updates. 
  This is because the bulk of the computation happens in 

The actions are defined the `autoware_auto_msgs` package. 

Inputs:

* Action call of type `autoware_auto_msgs/action/PlanTrajectory`

Outputs:

* Response to the above action call, as well as publication of planned trajectories.
  One trajectory is published per action call.

## Complexity

See `parking_planner`.

# Security considerations 

TBD by a security specialist.

# Future extensions / Unimplemented parts

* Proper `tf` support
