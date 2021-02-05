ray_ground_classifier {#ray-ground-classifier-design}
=====================

# Purpose / Use cases

Autoware.Auto requires a method to distinguish ground points from non-ground points in point clouds.
This is meant to act as a filtering step for object detection algorithms
(working on non-ground points), and free space estimation algorithms (working on ground points).
This method is very fast and allows the filtered point cloud to be exposed while the packets for a
scan are still coming in.


# Design

## Algorithm Design

Concretely, the original implementation of the ray ground filter batches points in a pointcloud
according to azimuth slice (that is, the angle between x and y in the vehicle frame). These are
typically binned into angular segments of 0.1 degrees rad. This roughly corresponds to one firing
of the VLP-16's.

Once the point cloud is restructured into rays, each point in a ray is annotated with the planar
distance (euclidean distance on x-y place only, again in vehicle frame). Points with height too
high, or radius too low are ignored. The ray is then sorted in order of increasing radial distance.

Once the rays are ordered, the algorithm scans through the points in the ray in order of increasing
radial distance. The algorithm defines two cones: a global cone and a local cone, where the local
cone has a wider aperture than the global cone. The global cone is rooted at the point cloud's
footprint  (e.g. projected down to the ground plane). The local cone is rooted at the last point
processed.

The ray ground filter has two main state variables: the last point processed, and whether the last
point was labeled as a ground point. These variables are initialized as the root of the global
cone and false (not ground) initially. For each subsequent point, the following logic is then used
to label the point:

```
If current point is within the local cone:
    If last was ground, this is also ground
    Otherwise, ground only if current point is in the global cone
Otherwise ground only if current point is within the global cone, and if the radial distance from
the last point is above a threshold.
```

The first branch gives near points (in the z sense) the same label.
The second branch provides a mechanism to switch back to ground points. In the last branch, the
current point is already outside of the local cone (in the z sense), so it must first be nonlocal
(in the r sense) before it can be checked against the global cone.


## Software Design Decisions

A new point structure was added. This was added to precompute the projected radius
of a point, and save a little computational work during the processing of a ray (i.e. labeling,
sorting).

## Inputs / Outputs / API

Inputs:

A block of points with at least the following fields:

- `x`
- `y`
- `z`
- `ray id`

Such that all points with the same ray are contiguous. Within a single block,
`ray id` may not repeat in non contiguous sets.


Outputs:

Two output blocks of points (one for ground, and one for not ground) with at
least the following fields:

- `x`
- `y`
- `z`
- `ray id`

Additionally, streams are delineated by uninitialized points with the spatial
`ray id` `velodyne::END OF SCAN ID`.

For more details, see the
[API documentation](@ref autoware::perception::filters::ray_ground_classifier::RayGroundClassifier).


## Inner-workings / Algorithms

The above stated algorithm was modified to work directly on the point stream
coming from the device and parsed by the (Velodyne) driver. Autoware.Auto does the following:

- Each point is assigned a `ray id` at the driver level according to its
position in the LiDAR data packet
    - A single firing of the LiDAR is considered to be one ray
    - The packet is parsed sequentially so that all points in a ray are
    contiguous in the outgoing point stream
- A stream of points is read in by the ray ground filter
    - As points come in, the radius is computed and the point is stored in an
    array
- Once the `ray id` of a new point is different from the ID currently being
tracked, the buffer of points that make of the current ray is sorted, and passed
through the above logic in radial order, or in increasing height in the case of
equal radial distance
    - `std::partial_sort` is used for sortingrather than `std::sort` or
    `std::stable_sort` even though it is less performant because it is not recursive
- As each point is labeled, it is pushed to the appropriate output buffer

In addition, to make classification of distant points more robust, a
limit is added to the global height threshold.  This was added to prevent the global
height threshold from growing unbounded with increasing distance, resulting in
most, if not all distant points to be classified as ground.

Additional rules were also added to improve the performance of the algorithm:

- If there is vertical structure with respect to the next point, the next point
and the last point are non-ground
- If the last point is nonground and the current point is near the last ground
point, then the current point is ground if the next point is ground (under the
  assumption that the current point is ground), otherwise it is nonground
- If the last point is nonground and the current point is vertically near the
last point, it is also nonground


# Performance characterization


## Time

The time complexity of filtering a single ray is `O(k + k log(k) + k)` for
inserting and calculating radius, sorting by radius, and then scanning through
points to label as ground or non-ground.

The operation is thus `O(k log k)`. `k` is the size of a laser scan ray, which
in this implementation is a hardware defined constant.

Filtering an entire point cloud of size `n` consequently takes `n log k` time.


### Space

The space complexity for this module is `O(k)=O(1)`, where `k` is the size of a
laser scan ray, which is a hardware defined constant.


## States


### Core states

The core filter has three floating point state variables fully determined by
the last point processed:

- `last_point_ground`
- `last_point_height`
- `last_point_radius`


### Configuration state

The following configuration state is introduced by the configuration class,
[Config](@ref autoware::perception::filters::ray_ground_classifier::Config).

# Error detection and handling

The main potential failure modes are due to undefined behavior from
`std::partial_sort`, e.g. if the comparator is semantically incorrect, and
indexing outside of an array. The former should be correct, and there are checks
to prevent the latter.

Additionally, the
[RayGroundClassifier](@ref autoware::perception::filters::ray_ground_classifier::RayGroundClassifier)
internally makes use of a pointer during a function call, and an array internally for storage.
These two elements bring the potential of errors, though accessing these elements is guarded by
proper checks.

Next, the primary invariant of the algorithm is that the core labeling function requires
points to come in order of increasing radial distance. The list is sorted by increasing height
if radial distance is approximately the same. This invariant is checked to not be broken
at the beginning of the core labeling method. It is worth noting that there is some threshold
to compensate for floating point error. The radial distance is allowed to shrink by as much
as [FEPS](autoware::common::types::FEPS) in order to handle the case when radial
distance is approximately equal. While this allows for an invariant to be broken, in practice
this should not cause issues, because if the maximum recession occurs for 1000 points, the net
recession would be no more than 1 mm, which is negligible, considering the VLP16-HiRes'
precision is no greater than 1 cm.

If configuration parameters are semantically incorrect with respect to one
another, then an error will be thrown during construction of the configuration class.


# Security considerations

TBD by security expert.


# References / External links

The design of the ray ground filter is based off an implementation provided in
[CPFL's Autoware](https://github.com/CPFL/Autoware/blob/7b384fbdc97793d8bf7c1387d22c50e1fc9109e3/ros/src/sensing/filters/packages/points_preprocessor/nodes/ray_ground_filter/ray_ground_filter.cpp)


# Future extensions / Unimplemented parts

Correction + estimation using global ground-plane estimate from the last frame.
