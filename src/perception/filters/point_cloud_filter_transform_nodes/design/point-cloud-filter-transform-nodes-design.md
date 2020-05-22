Point cloud filter transform nodes {#point-cloud-filter-transform-nodes}
=============

# Purpose / Use cases

To properly separate concerns and make it easier to reuse data (i.e. prerecorded bag data, or from the simulator),
we need to separate the transforming and filtering data from the drivers into separate nodes.

In particular, we require a boilerplate wrapper around our algorithm, which is implemented as a
library.
This boilerplate node wrapper allows us to communicate with the rest of Autoware.Auto.

# Design

There is one instance of these nodes for the
[PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg) message type for visualization and open source support.

The `point_cloud_filter_transform_node_exe` and `point_block_filter_transform_node_exe` are wrappers
around the `DistanceFilter`, `AngleFilter`, and `StaticTransformer` with the following executing order:

1. Check if the points are within the range of `DistanceFilter` and `AngleFilter`
2. Transform the points using `StaticTransformer`
3. Publish the transformed and filtered data in [PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg) format

## Assumptions / Known limits

The implementation doesn't allow dynamic thresholds for distance and angle filters.
It also doesn't support dynamic transformation during runtime.
The thresholds and the transform parameters can only be loaded while constructing the nodes.

## Inputs / Outputs / API

These nodes have the following basic structure:

Input:
- points_raw (Message of `PointCloud2` directly from driver nodes)

Output:
- points_filtered (Message of `PointCloud2`)

On top of this, the nodes can be configured either programmatically or via parameter file
on construction.

## Error detection and handling

Most error handling occurs inside `rclcpp`.

The only explicit error handling that occurs in these nodes is catching errors in the runtime loop.

If such an error is caught, then the node will simply transition to a "failed state", log this fact,
and attempt to continue running.

## Security considerations

TBD by a security specialist.

# Related issues

- #328: Port transform/filter node from Apex.Auto to Autoware.Auto; Remove filter/transform functionality from drivers
