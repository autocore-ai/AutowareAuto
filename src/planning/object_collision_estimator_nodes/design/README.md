Object Collision Estimator Nodes {#object-collision-estimator-nodes}
===========

# Purpose / Use cases

The collision estimator node is the interface for detecting collisions.
The behavior planner delegates the task of estimating if any collision is likely on a planned path to this node.

# Design

The node implements a ROS2 interface to Object Collision Estimator Library. It has 2 main external interfaces:

1. Subscribe to a topic to obtain obstacle bounding boxes from perception stack.
1. Present as a service to the behavior planner. This service takes a trajectory as an input and outputs another trajectory modified to avoiding collisions.

## Assumptions / Known limits

- The trajectory is published in map frame.
- A transform between the perception bounding box frame and map exists

## Inputs / Outputs / API

Inputs:

- `BoundingBoxArray.msg`
  - A list of bounding boxes of obstacles.
  - This is received on a topic determined by the node parameter `object_collision_estimator.obstacle_topic`
- `Trajectory.msg`
  - Local path of the ego vehicle given by the behavior planner.
  - Received on the service interface

Outputs:

- `Trajectory.msg`
  - Modified local path to avoid any collisions
  - Returned on the service interface

## Inner-workings / Algorithms

- Obstacle Subscriber
  - Subscribes to the obstacle topic which gives a list of bounding boxes representing obstacles detected by the perception pipeline.
  - The boxes are transformed into the map frame.
  - The boxes are then passed to the ObjectCollisionEstimator object.
- Collision estimation service
  - Gets a request containing a planned trajectory from the behavior planner.
  - Pass this trajectory to ObjectCollisionEstimator who modifies it to avoid any collision.
  - Return the modified trajectory to caller of the service.

## Error detection and handling

Obstacles that have bounding-box edges too small for the geometry computations get their size
increased to the minimum value defined by
[min_obstacle_dimension_m](@ref motion::planning::object_collision_estimator::ObjectCollisionEstimatorConfig::min_obstacle_dimension_m).
This node emits a warning each time such an obstacle is encountered.

# Future extensions / Unimplemented parts

- If a transform between bounding box and map does not exist, the estimator should return empty trajectory and the vehicle should stop.
- Test tf transform code paths.
- Vary collision detection method and tolerances based on vehicle operation mode.

# Related issues

- #474: Estimate collisions based on detected objects and vehicle path (Object Collision Estimator)
- #447: Implement Semantic-Map-Based Navigation and Planning
