euclidean_cluster_nodes {#euclidean-cluster-nodes-design}
=======================

# Purpose / Use cases

An autonomous driving stack needs some notion of where (hittable) objects are in Cartesian space.
This is necessary so that planners (e.g. motion, behavior) can plan safe trajectories and behaviors
that do not result in anything dangerous occurring, such as hitting an object.

The
[euclidean clustering](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster)
algorithm aggregate non-ground points into logical clusters, which roughly represent an object.
These clusters can be used directly or converted to an intermediate representation for further use.

The euclidean clustering algorithm for object detection must be wrapped in a node to communicate
and integrate with a larger system or stack.

# Design

As a terminal algorithm in a stack, the euclidean clustering algorithm should be highly configurable
as many behaviors are allowed and permissible. In particular, switches or configurations should
permit the following behaviors:

1. Accept inputs (i.e. PointCloud2)
2. Optionally apply downsampling to the input
3. Clustering is assumed to happen in all cases
4. Optionally publish the resulting clusters
5. Optionally compute and publish bounding boxes around the clusters as either `DetectedObjects` 
   type or `BoundingBoxArray` type
6. Optionally apply aggressive downsampling (or not) and ignore the z direction entirely

## Assumptions / Known limits

The euclidean cluster node assumes a bounded capacity. If this capacity
is overrun, then an exception will be thrown, dropping the current frame.

If an exception is thrown during the bounding box/hull formation process, then the box
in the process of being formed will be lost.

The block version has no frame information, and as such assumes that all of the input
data comes from the same coordinate frame.

Currently, this component is not static because the following subcomponents are not static:
- spatial_hash (euclidean clustering algorithm)
- voxel_grid
- Node pub/sub

## Inputs / Outputs / API

As a node, this component's API is largely expressed by interprocess communication handles,
e.g. topics. For more details on the API, see the
[docs](@ref autoware::perception::segmentation::euclidean_cluster).

### Inputs

The euclidean cluster nodes as a family accept one input: non-ground points as a
PointCloud2 message.

### Outputs

Any combination of the following three outputs are allowed (except using neither):

1. [PointClusters](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/raw/master/autoware_auto_msgs/msg/PointClusters.msg)
2. [BoundingBoxArray](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/raw/master/autoware_auto_msgs/msg/BoundingBoxArray.msg)
3. [DetectedObjects](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/raw/master/autoware_auto_msgs/msg/DetectedObjects.msg)

The first represents the raw clustering output of the object detection algorithm.

The other two represent the processed output of the resulting clusters, which is somewhat
more compact and simpler to interpret and handle. They are provided mainly for the purposes of memory efficiency.

### Parameters
- `use_detected_objects` - When true, the node publishes bounding boxes as a `DetectedObjects` msg.
- `use_cluster` - When true, the node publishes clusters as `PointClusters` msg; otherwise, clusters are not published.
- `use_box` - When true, clusters are formed into a box shape and published as a `BoundingBoxArray` msg.
- `max_cloud_size` - Maximum number of points expected in the input point cloud. Used to preallocate internal types.
- `downsample` - Parameter to control whether to downsample the input point cloud using a voxel grid. If this is set to true, a set of `voxel` parameters need to be defined.
- `use_lfit` - When true, the `L-fit` method of fitting a bounding box to cluster will be used; otherwise,the  `EigenBoxes` method will be used.
- `use_z` - When true, height of bounding boxes will be estimated; otherwise, height will be set to zero.

@note At least one of `use_cluster`, `use_box`, and `use_detected_objects` has to be set to true.

In addition to the above params, clustering parameters are also needed to run this node; they correspond to the members of the `Config` class in the `euclidean_cluster` package. The names are prefixed with `cluster.`.
Furthermore, spatial hashing parameters `hash.min_x`, `hash.max_x`, `hash.min_y`, `hash.max_y`, `hash.side_length`, `max_cloud_size` are required. See the documentation on spatial hashing for information.


## Error detection and handling

In general, exceptions are thrown if something unexpected happens.

There are two exceptions to this rule:

1. If an exception is thrown due to an `std::length_error`, i.e. the underlying data structure hit
capacity, then the clustering process immediately starts
2. If during the clustering process an error occurs, e.g. the maximum number of clusters would be
exceeded, then an exception is thrown only after the clustering process and publishing

The reasons for these exceptions is because while the clustering algorithm and its result
may not be in a good state, there is still validity to the information, if incomplete. As such,
the most recent information should be made available, and hopefully tracking should cover any gaps.

## Security considerations

TBD by a security specialist.

# Future extensions / Unimplemented parts

- Concave cluster decomposition
- Static memory
- Fix bounding box intersection errors


# Related issues

- #28: Port to open source
