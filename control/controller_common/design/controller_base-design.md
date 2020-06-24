Controller Base Design {#controller-base-design}
=============================================

# Purpose

The purpose of this base class is to encapsulate common functionality of a controller.

This common functionality is then intended to ease the implementation of specific controller
algorithms.

# API

## Implementing Child Classes

`compute_command_impl(State)` must be implemented. This defines the control behavior of the
controller under nominal trajectory following conditions.

`check_new_trajectory(Trajectory)` can be overridden to allow the user to define whether
a trajectory is accepted or rejected. Some examples can include checking if the provided
trajectory meets certain assumptions required for the control algorithm to work, such as
a specified sampling rate.

`handle_new_trajectory(Trajectory)` can be overridden to allow the user to update book-keeping,
such as for an algorithm-specific representation of the trajectory.

# Design

Some input validation is baked into the design. `compute_command` will throw an exception
if the state is not in the same frame as the trajectory. It is the user's responsibility
to ensure that the inputs are compatible.

# Behaviors

A number of basic behaviors are intended to be implemented by the base controller class
so that algorithm implementers need not worry about the implementation. In addition,
this will ensure that all controllers implementing this interface exhibit the same behavior
during edge cases.

The following behaviors are implemented:
- Safe stopping behavior
- Reference point tracking

## Safe stopping behavior

When a trajectory is not present, or otherwise unavailable, the vehicle should come to a smooth
stop.

Control commands that will bring the vehicle to a smooth stop will be generated in lieu of
an algorithm computing the commands under the following conditions:
- If no trajectory is provided
- An empty trajectory is provided
- If the provided vehicle state is beyond the trajectory in time or space

## Reference point tracking

On each call to `compute_command(State)`, the reference index with respect to the trajectory is
updated. **The reference index is the last point the given state is past** (or equal to).

- The reference index can only increase with subsequent calls to `compute_command(State)`
- The reference index is reset to 0 on calls to `set_trajectory(Trajectory)` variants

Different algorithms work on spatial or temporal references. As such, an API for both tracking
the current state in both the time and space sense is provided.
