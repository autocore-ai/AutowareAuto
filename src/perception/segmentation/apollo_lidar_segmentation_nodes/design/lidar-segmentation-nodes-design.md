apollo_lidar_segmentation_nodes {#apollo-lidar-segmentation-nodes-design}
========================

# Purpose / Use cases

An alternative to Euclidean clustering.
This node detects and labels foreground obstacles (e.g. cars, motorcycles, pedestrians) from a point
cloud, using a neural network.

# Design

See the design of the algorithm in the core package's [design](@ref apollo-lidar-segmentation-design).

## Assumptions / Known limits

The original node from Apollo has a Region Of Interest (ROI) filter.
This has the benefit of working with a filtered point cloud that includes only the points inside the
ROI (i.e., the drivable road and junction areas) with most of the background obstacles removed (such
as buildings and trees around the road region).
Not having this filter may negatively impact performance.

## Inputs / Outputs / API

### Inputs

The input are non-ground points as a PointCloud2 message from the sensor_msgs package.

### Outputs

The output is a [BoundingBoxArray](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/raw/master/autoware_auto_msgs/msg/BoundingBoxArray.msg).

### Parameters

|Parameter|Type|Description|Default|
|---------|----|-----------|-------|
|`range`|*int*|The range of the 2D grid with respect to the origin (the LiDAR sensor). Unit: meter|`70`|
|`score_threshold`|*float*|The detection confidence score threshold for filtering out the candidate clusters in the post-processing step.|`0.8`|
|`use_intensity_feature`|*bool*|Enable input channel internsity feature.|`true`|
|`use_constant_feature`|*bool*|Enable input channel constant feature.|`false`|
|`z_offset`|*float*|The offset to translate up the input pointcloud before the inference. Unit: meter|`0.0`|
|`min_height`|*float*|The minimum height with respect to the origin (the LiDAR sensor). Unit: meter|`-5.0`|
|`max_height`|*float*|The maximum height with respect to the origin (the LiDAR sensor). Unit: meter|`5.0`|
|`objectness_thresh`|*float*|The threshold of objectness for filtering out non-object cells in the obstacle clustering step.|`0.5`|
|`min_pts_num`|*int*|In the post-processing step, the candidate clusters with less than min_pts_num points are removed.|`3`|
|`height_thresh`|*float*|If it is non-negative, the points that are higher than the predicted object height by height_thresh are filtered out in the post-processing step. Unit: meter|`0.5`|

## Error detection and handling

Abort and warn when the input frame can't be converted to `base_link`.

## Security considerations

Both the input and output are controlled by the same actor, so the following security concerns are
out-of-scope:

- Spoofing
- Tampering

Leaking data to another actor would require a flaw in TVM or the host operating system that allows
arbitrary memory to be read, a significant security flaw in itself.
This is also true for an external actor operating the pipeline early: only the object that initiated
the pipeline can run the methods to receive its output.

A Denial-of-Service attack could make the target hardware unusable for other pipelines but would
require being able to run code on the CPU, which would already allow a more severe Denial-of-Service
attack.

No elevation of privilege is required for this package.

# Future extensions / Unimplemented parts

The output is a `BoundingBoxArray`.
It could be converted to a `DetectedObstacle` the same way the
[euclidean_cluster_nodes](@ref autoware::perception::segmentation::euclidean_cluster_nodes) package
is doing it, possibly by using the same functions.

# Related issues

- #677: Port apollo segmentation detect to autoware.auto
