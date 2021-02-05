ray_ground_classifier_nodes {#ray-ground-classifier-nodes-design}
===========================

# Purpose / Use cases

We need a node that partitions raw groups of points into ground and nonground sets.
This initial preprocessing allows us to more efficiently use downstream algorithms.

In particular, we require a boilerplate wrapper around our algorithm, which is implemented as a
library. This boilerplate node wrapper allows us to communicate with the rest of Apex.OS.

# Design

These nodes are simple wrappers around the
[`RayGroundClassifer`](@ref autoware::perception::filters::ray_ground_classifier::RayGroundClassifier).

A node for the `PointCloud2` message type is provided for visualization and open source support.
The `PointCloud2` node supports unstructured point clouds.

An input is considered structured if the ray information needed for the classifier to work is
known apriori (e.g. encoded in how the points are received from the sensor).

By contrast, an input is considered unstructured if ray information is not known apriori.
This then means that extra work must be done to determine this information. Point clouds
are considered by default to be unstructured.

As such the `RayGroundClassifierCloudNode` has an instance of the `RayAggregator` to provide
structure to the unstructured point clouds.


## Assumptions / Known limits

The current assumption is that the inputs will be structured. This assumption will be
dropped in the future.

Another limitation is that the `RayGroundClassifier` can maximally handle rays of
512 in size.

Finally, to simplify logic, ray id sequences must be contiguous, e.g. a sequence of:

```
1-1-2-2-1-1
```

is not recognized as two rays, but rather three.

See the
[RayGroundClassifier](@ref ray-ground-classifier-design)
design doc for more details.

## Inputs / Outputs / API

These nodes have the following basic structure:

Input:
- raw_points (Message of PointBlock or PointCloud2 directly from driver nodes)

Outputs:
- points_ground (Message of PointBlock or PointCloud2)
- points_nonground (Message of PointBlock or PointCloud2)

On top of this, the nodes can be configured either programmatically or via parameter file
on construction.


## Error detection and handling

Most error handling occurs inside `rclcpp`, inside the `RayGroundClassifier`
itself, or in the associated configuration class.

The only explicit error handling that occurs in these nodes is catching errors in the runtime loop.

If such an error is caught, then the node will simply transition to a "failed state", log this fact,
and attempt to continue running.

## Security considerations

These components inherit security considerations from `rclcpp::Node`, and
the core `RayGroundClassifier` class.


# Future extensions / Unimplemented parts

- Cleanup to remove refactor boilerplate

# Related issues

- #2066: Initial implementation
- #1956: Replaced apex time with `std::chrono` constructs.
- #1935: Add unstructured behavior to block node, add cloud node
- #2150: Harvest header info for structured_block callback handling
