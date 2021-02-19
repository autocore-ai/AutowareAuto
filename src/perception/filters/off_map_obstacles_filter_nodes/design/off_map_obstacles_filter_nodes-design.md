off_map_obstacles_filter_nodes {#off-map-obstacles-filter-nodes-package-design}
===========

This is the design document for the `off_map_obstacles_filter_nodes` package.

# Purpose / Use cases
This is a temporary workaround for limitations in the parking planner and obstacle detection.
Namely, some obstacles that were off the map (often spurious detections) were interrupting the parking maneuver.

The true solution would be to produce detections only for actual dynamic objects, and for the parking planner to be more robust (e.g. resume parking after waiting for an obstacle to disappear).

# Design
This is a standard ROS-level wrapper for the pure `off_map_obstacles_filter` package.

It fetches a lanelet map, and filters all incoming `BoundingBoxArray` messages. To calculate their overlap with the map, it also transforms them from their native frame into the map frame by asking `tf2_ros` for a transform with the same timestamp.


## Assumptions / Known limits
It is assumed that the transform is available within 100ms.

## Inputs / Outputs / API
The input topic is `bounding_boxes_in`, the output topic is `bounding_boxes_out`. All settings are done through parameters.

## Error detection and handling
If a transform is not available, the obstacles are not filtered because that means the system will fall back to braking for potentially spurious obstacles.

# Future extensions / Unimplemented parts
The map loading could be a bit nicer, also in the `lanelet2_map_provider` itself. But in the future, we should remove this node entirely anyway.

# Related issues
Issue #840.
