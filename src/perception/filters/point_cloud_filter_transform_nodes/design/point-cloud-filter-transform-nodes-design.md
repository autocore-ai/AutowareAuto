point_cloud_filter_transform_nodes {#point-cloud-filter-transform-nodes}
==================================

@tableofcontents

# Purpose / Use cases

To properly separate concerns and make it easier to reuse data (i.e. prerecorded bag data, or from the simulator),
we need to separate the transforming and filtering data from the drivers into separate nodes.

In particular, we require a boilerplate wrapper around our algorithm, which is implemented as a
library called `lidar_utils`.
This boilerplate node wrapper allows us to communicate with the rest of Autoware.Auto.

# Design

There is one instance of these nodes for the
[PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg) message type for visualization and open source support.

The following components are parts of the `lidar_utils` package but they are used by the 
`point_cloud_filter_transform` node to filter and transform the input point cloud data : 

## Distance Filter
Filter class to check if a point is contained in a range defined by a minimum and maximum radii from the origin. Operates in the square forms to avoid using sqrt().

## Angle filter
Filter class to check if a point lies within a range defined by minimum and maximum angles. The 
inclusion zone is defined by the range  [start angle , end angle] in counter-clockwise 
direction. To avoid explicitly computing the angle using inverse trigonometric functions, the following scheme is used:
- A vector called `range_normal` is defined to be the unit vector placed in the median of the range.
- The unit vector at the min or max angle is projected to this range_normal . The 
  projection is now a factor of the cosine of the half of the accepted angle range \f${\theta}
  \f$ which will be (\f$cos(\frac{\theta}{2})\f$). This value is called the threshold.
- If a point is within the range, value of its projection on the range normal should be 
  greater-equal than this threshold value. The filter operates in square form to avoid using sqrt()

## Static Transformer
Applies a transform to given points. Uses the Eigen library during computations.


The `point_cloud_filter_transform_node_exe` is a wrapper
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
- points_xyzi (Message of `PointCloud2` directly from driver nodes or the `point_type_adapter node`)

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
