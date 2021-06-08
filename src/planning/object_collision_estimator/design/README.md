Object Collision Estimator {#object-collision-estimator}
=========================

# Purpose / Use cases

The collision estimator takes a list of objects detected by the perception stack and the local path produced by the local planner as inputs.
It then predicts any collisions between the ego vehicle and static obstacles.
Finally the local path is modified to avoid any potential collisions.

# Design

## Inputs / Outputs

Inputs:

- `BoundingBoxArray.msg`
  - A list of bounding boxes of obstacles
  - Produced by the perception stack
- `Trajectory.msg`
  - Local Path of the ego vehicle
  - Produced by Local Planner

Outputs:

- `Trajectory.msg`
  - Modified Local Path
  - Modified to avoid any collisions

## Algorithms

This is the workflow of the estimator:

- Receive a list of obstacles.
- Increase the size of the obstacles that are too small.
- Receive a trajectory.
- Loop trough the points on the trajectory.
- For each point, create a bounding box representing the volume occupied by the ego vehicle at that point.
- For each obstacle, detect if there is overlap between the obstacle bounding box and the ego vehicle bounding box.
- If overlap detected, curtail the trajectory to the point just before the collision. Set the velocity and acceleration of the last point to zero.
- Pass the trajectory to a smoother to make the velocity profile more smooth.
- The smoother sets the velocity of the last few points to zero.
- Then it passes the velocity profile through a gaussian filter thus ending up with a velocity profile that gradually ramps down to zero.

## Assumptions / Known limits

- The obstacles are in the same coordinate frame as the trajectory.
- The bounding boxes are assumed to be parallel to the ground and is represented by the rectangle of the bottom surface and z = 0. Hence the collision detection only happens in 2d.
- The corners of the bounding boxes are expected to be in counterclockwise order.
- The x and y elements of the `size` object are expected to map to the length of the first and second edges respectively.

## Error detection and handling

All API should not emit exceptions.
If collision detection algorithm fail in any way, an empty trajectory should be returned and the behavior planner should perform emergency stop.

# Related issues

- #474: Estimate collisions based on detected objects and vehicle path (Object Collision Estimator)
- #447: Implement Semantic-Map-Based Navigation and Planning
