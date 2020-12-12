RecordReplay planner nodes {#recordreplay-planner-nodes}
==========================

# Purpose / Use cases

This package provides a ROS2 node for the computational package `recordreplay_planner`. This is done to
separate the computation part from the ROS2-specific part. See the computational part for a rationale of what
the functionality is to be used for.


# Design

This is a wrapper around `recordreplay_planner`. Its behavior can be controlled via actions. It can record
the ego state of the vehicle and play back a set of recorded states at a later time. It does stop for obstacles, 
from `BoundingBoxArray`, publsihed in any `tf` frame that can transform to `VehicleKinematicState`'s frame.


## Assumptions / Known limits

This node assumes that the recorded states can be played back without any `tf` changes. There is an open issue
in https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/issues/265 to address this limitation.

See the `recordreplay_planner` design documentation for more details.


## Inputs / Outputs / API

Actions:

This node uses two actions to control its behavior:

* `RecordTrajectory.action` is used to record a trajectory. It runs until canceled. While the action is
  running, the node subscribes to a `VehicleKinematicState.msg` topic by a provided name and records all
  states that are published on that topic. 
* `ReplayTrajectory.action` is used to replay a trajectory. It runs until canceled. While the action is 
  running, the node subscribes to the same `VehicleKinematicState.msg` topic as when recording. When messages
  are published on that topic, the node publishes a trajectory starting approximately at that point (see the
  `recordreplay_planner` design documentation on how that point is determined).  

The actions are defined in a separate package, `recordreplay_planner_actions`.

Inputs:

* `autoware_auto_msgs/msg/VehicleKinematicState` is the state used as recorded points for replaym, and also to prune starting point of replay trajectory
* `autoware_auto_msgs/msg/BoundingBoxArray` is list of bounding boxes of obstacle, that thre reaply trajectory stops for to avoid collision.

Outputs:

* `autoware_auto_msgs/msg/Trajectory` is the trajectory that gets published

## Complexity

See `recordreplay_planner`.

# Security considerations 

TBD by a security specialist.

# Future extensions / Unimplemented parts

* Proper `tf` support

# Related issues

* #265 Extension to support `tf` properly
* #273 Fix and extend integration tests
