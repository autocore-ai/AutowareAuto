recordreplay_planner
====================

# Purpose / Use cases

This package provides a simple class for recording states, then playing them back as a trajectory. This is to
be used in both simulator as well as on-site tests of the controller: One can turn on recording, start driving
a trajectory, then reset vehicle position, then start replay and see what the controller does.


# Design

Recording will add states at the end of an internal list of states.

The replay will find the closest state in terms of location and heading along the recorded list of states,
and deliver trajectories starting from that state. The trajectory length is at most 100 as specified by the
`Trajectory` message, and at least 1 if there is any recorded data present.


## Assumptions / Known limits

There is no interpolation between points along the trajectory, and localization is not done in a smart way:
The list of recorded states is simply iterated over.

## Inputs / Outputs / API

Inputs:

* `VehicleKinematicState.msg` is the state that gets recorded

Outputs:

* `Trajectory.msg` is the trajectory that gets published


## Complexity

Recording is `O(1)` in time and `O(n)` in space, replay is `O(n)` in both time and space, where `n` is the
number of recorded states.

# Security considerations 

TBD by a security specialist.

# Future extensions / Unimplemented parts

* Trajectory buffer clearing
* Proper `tf` support

# Related issues

