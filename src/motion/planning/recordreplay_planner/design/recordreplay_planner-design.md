recordreplay_planner
====================

# Purpose / Use cases

This package provides a simple class for recording states, then playing them back as a trajectory. This is to
be used in both the simulator as well as in on-site tests of the controller: One can turn on recording, start
driving a trajectory, then reset vehicle position, then start replay and see what the controller does.

It also incorporates a limited capability to try and avoid collisions.


# Design

Recording will add states at the end of an internal list of states.

The replay will find the closest state in terms of location and heading along the recorded list of states, and
deliver trajectories starting from that state. The trajectory length is at most 100 as specified by the
`Trajectory` message, and at least 1 if there is any recorded data present.

A list of obstacles can also be specified via a method. Every trajectory point is checked for collisions with the
currently stored list of obstacles. Trajectory points are converted cached as ego bounding boxes with ego vehicles 
dimensions for collision checking. If any trajectory box is found to collide, the trajectory is cut to end at one
state before the colliding state, and the desired velocity for the end of the trajectory is set to 0.  No
effort is currently made to create a dynamically feasible velocity profile.

## Assumptions / Known limits

There is no interpolation between points along the trajectory, and localization is not done in a smart way:
The list of recorded states is simply iterated over.

The stopping concept only works if one can assume that the downstream controller and the vehicle are able
to track the desired velocity going to zero in a single trajectory step. 

Making sure this assumption is satisfied by construction would involve creating a dynamically feasible
velocity profile for stopping - this has not been done yet. 

## Inputs / Outputs / API

Inputs:

* `VehicleKinematicState.msg` is the state that gets recorded
* `BoundingBoxArray.msg` is a list of bounding boxes of obstacles

Outputs:

* `Trajectory.msg` is the trajectory that gets published


## Complexity

Recording is `O(1)` in time and `O(n)` in space, replay is `O(n)` in both time and space, where `n` is the
number of recorded states. Collision checking currently happens on every replay even if obstacles do not
change, and has a complexity that is linear in the number of obstacles but proportional to the product of 
the number of halfplanes in the ego vehicle and a single obstacle.

# Security considerations 

TBD by a security specialist.

# Future extensions / Unimplemented parts

* Trajectory buffer clearing
* Proper `tf` support
* rosbag2 support for recording and later replaying

# Related issues

