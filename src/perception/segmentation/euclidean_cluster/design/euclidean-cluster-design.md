euclidean_cluster {#euclidean-cluster-design}
=================


Abstractly, euclidean clustering groups points into clusters such that for any
two points in a cluster, there exists a chain of points also within that cluster
between both points such that the projected distance between subsequent points
in the chain is less than some threshold.

Projected distance is the distance
between two points projected onto the xy plane in the vehicle frame. Clusters
are valid objects if the number of points in the cluster is above a given
threshold.

Another way of looking at the result is that a connectivity graph is generated,
and clusters are connected components with more than a certain number of
vertices.

Vertices in this view are point cloud points, and edges are drawn between points
with a project distance below some threshold.

Concretely, the algorithm to construct these clusters works in the following way:

```
The incoming downsampled point cloud is partitioned into concentric rings
Each ring is stored in a different kd-tree
For each ring, the following procedure is run with a different distance
threshold:
   For each point in the pointcloud:
       If seen, continue
       Begin new cluster and add point to queue
       While queue is not empty, pop point:
           Find each near neighbor of point, if not seen, add to queue
   Add point to cluster and mark point as seen
Merge: for each pair of clusters, if the distance between centroids is below a
threshold, merge clusters
Merge: Apply the above procedure on the result of the above
Dump resulting point clouds to message.
```


# Modifications

The following modifications were made for a simplified algorithm and higher speed:

- Near neighbor queries were done using a 2D spatial hash (similar to a voxel
grid) for faster near neighbor queries
- The input into the spatial hash was the output of a 2D voxel grid with a
maximum height of approximately the vehicle's height. This is to reduce the
number of points into the spatial hash and reduce computational burden

# Performance characterization


## Time

The algorithm consists of the following steps:
- Insert `n` points into spatial hash (`O(n)` to insert): `O(n^2)`
- For each of the `n` points that are unseen:
   - Iterate over `k` near neighbors (`O(1)` lookup time, though it is possible
   to construct adversarial `O(n)` examples)
       - Assign near neighbor to cluster, mark as seen, remove from spatial hash

This results in an algorithm that is `O(n^2 + n^2)`, but is in practice `O(n)`.
In practice, the first term should be `O(n)` as inserting into a hashmap is
average case `O(1)`.

## Space

The module consists of the following components:
- Spatial hash: `O(n)`:
    - The internal hashmap is `O(n + n + A * n)`, where `A` is an arbitrary
    constant (load factor)
    - The other components of the spatial hash are `O(n + n)`
- Other components: `O(n + B * n)` where `B <= 1` is a constant factor
(1 / points per cluster)

This results in `O(n)` space complexity.

## States

Conceptually, the clustering algorithm has three states:

1. Point acquisition (e.g. multiple
[insert](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster::insert)
calls are occurring in a sequence)
2. Clustering (i.e. a call to one of the
[cluster](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster::cluster)
methods)
3. Result ready (i.e. before
[cleanup](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster::cleanup)
is called)

The second state, clustering, can be thought of as a separate state because it performs destructive
operations on the underlying data structure.

By contrast, the first state, point acquisition, performs constructive (additive) operations on the
underlying data structure.

Finally, the result ready state occurs between clustering and cleanup. This state is available
such that operations or manipulations may be done on the resulting (large) cluster data.

## API

For more details on the API, see the
[documentation](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster).

This class supports two modes of output:

1. Using the internally allocated output
2. Using externally allocated output

The latter case is when further operations must be done on the output, such as sorting during a
hull formation process. In this case, preallocated memory must be explicitly returned to the
class via the
[cleanup](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster::cleanup)
method.

Finally,
[cleanup](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster::cleanup)
and
[get_error](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster::get_error)
are provided separate from
[cluster](@ref autoware::perception::segmentation::euclidean_cluster::EuclideanCluster::cluster)
because some errors during the clustering process do not necessarily invalidate the remainder
of the results. Recovery action should still be taken, but to some extent, the result is still
valid, if incomplete.

# Reference

Euclidean clustering is based off a core algorithm provided in [pcl](http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php)
with modifications from [CPFL's Autoware](https://github.com/CPFL/Autoware/blob/efe17abbb7b831d0e826fb881c365f9ce34cf1d4/ros/src/computing/perception/detection/packages/lidar_tracker/nodes/euclidean_cluster/euclidean_cluster.cpp).


# Related issues

- #28: Port to open source
