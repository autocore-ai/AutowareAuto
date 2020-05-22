Trajectory spoofer {#trajectory-spoofer-design}
===========

This is the design document for the `trajectory_spoofer` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package provides a ROS2 node and class for generating trajectories that can be used for testing as a replacement for motion planning.
This is done to be able test components that need a valid trajectory without being dependent on a motion planner or localization.
Additionally, this module gives control over the type of trajectory generated and provides repeatability, unlike using a motion planner.

This package is intended to be used to test a controller **in simulation only**.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
This node listens to a vehicle state and uses the first position received as the starting point in the trajectory.
Based on parameters, the node uses the `trajectory_spoofer` class to generate a trajectory.
Trajectories are based on simple geometric patterns like a straight line or circle.

## Assumptions / Known limits
<!-- Required -->
Trajectory will use the same TF frame as the incoming state message.

## Inputs / Outputs / API
<!-- Required -->

Parameters:

* speed_ramp_on (bool) : Currently not used
* target_speed (float): Speed used in all trajectory points
* num_of_points (int): Number of points to generate on the trajectory (max 100)
* trajectory_type (string): Currently supports only 'straight' or 'circle'
* length (float): Length of trajectory in meters, only used for `trajectory_type`='straight'
* radius (float): Radius of trajectory in meters, only used for `trajectory_type`='circle'

Inputs:

* `autoware_auto_msgs/msg/VehicleKinematicState` is the state that is set as the starting point and also triggers trajectory publishing

Outputs:

* `autoware_auto_msgs/msg/Trajectory` is the trajectory that gets published

Launch:

 * `$ros2 launch trajectory_spoofer trajectory_spoofer.launch.py`

## Inner-workings / Algorithms
<!-- If applicable -->


## Error detection and handling
<!-- Required -->

Not meant for production use.

# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

Not meant for production use, only for limited testing and only in simulation.

# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->
 * Add other types of geometry patterns like curvature, figure-eight, lane change, right/left turn, U-turn, etc.
 * Add different velocity profiles like ramping up, ramping down, ramping up and down, oscillatory, sudden brake, etc.

# Related issues
<!-- Required -->

* #233 Create dummy trajectory spoofer
