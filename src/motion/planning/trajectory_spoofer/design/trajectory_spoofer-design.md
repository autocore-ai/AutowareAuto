trajectory_spoofer {#trajectory_spoofer-package-design}
===========

This is the design document for the `trajectory_spoofer` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package provides a ROS2 node and class for generating trajectory that can be used for testing and development as replacement of motion planning.
This is intended to be used to test controller in simulation, or empty big parking log kind of real world testing.

This is done to be able test components and integration that needs a valid trajectory without being dependent on a motion planner and map based localization.
Additionally this module gives control over type of trajectory generated and repeatability, unlike using motion planner.
It could generate something very simple to something that challenges the controller and pushes it to it's limits.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
This node listens to vehicle state, and uses the first state as the starting point in the trajectory,
and then generates a trajectory. Based on parameters the node uses the `trajectory_spoofer` class to generate trajectories.
Trajectories are generated based on simple geometric patterns, like points on a straight line or circle.

## Assumptions / Known limits
<!-- Required -->
Trajectory can use the same tf frame as the incoming state message.

## Inputs / Outputs / API
<!-- Required -->

Parameters:

* speed_ramp_on (bool) : Not used
* target_speed (float): value using in all trajectory points
* num_of_points (int):  number of points to generate (max 100)
* trajectory_type (string): Currently supports only 'straight' or 'circle'
* length (float) : length of trajectory in meters, only used for trajectory_type='straight'
* radius (float) :  radius of trajectory in meters, only used for trajectory_type='circle'

Inputs:

* `VehicleKinematicState.msg` is the state that is set as the starting point and  also triggers trajectory publishing

Outputs:

* `Trajectory.msg` is the trajectory that gets published

Launch:

 * `$ros2 launch trajectory_spoofer trajectory_spoofer.launch.py`

## Inner-workings / Algorithms
<!-- If applicable -->


## Error detection and handling
<!-- Required -->

Not meant for production use

# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

Not meant for production use, only for limited deliberate testing and mostly in simulation

# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->
 * Add other types of geometry patterns like, curvature, figure eight, lane change, right/left turn, U-turn.
 * Add velocity different profiles, like ramping-up, slowing down, ramping up and down, oscillatory, sudden brake.

# Related issues
<!-- Required -->

* #233 Create dummy trajectory spoofer
