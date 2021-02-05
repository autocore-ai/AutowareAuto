ray_aggregator {#ray-aggregator-design}
==============

# Purpose / Use cases

The
[RayGroundClassifier](@ref autoware::perception::filters::ray_ground_classifier::RayGroundClassifier)
operates on Rays individually. This presupposes that ray information is
provided to the module. While this can be true for scanning LiDAR, in practice, this assumption is
not always available depending on interface definitions, driver implementation and LiDAR scan
patterns.

As such, a component that generates rays from an unstructured point clouds for use in ground
classification is needed to allow the `RayGroundClassifier` to work more generally.


# Design

The ray aggregator generally has three operations:

1. Add points to the aggregator
2. Check if any rays are ready
3. Get next ray

The second and third operations were separated out from a single operation to generally make the API
more concrete and immediately understandable.

## Algorithm Design

This object in general operates in the following manner:

1. Buffers are preallocated according to configuration settings
2. For each point an angle and thus bin is calculated, the point is added to the respective ray
3. If any ray satisfies a given criterion, then it (it's index) is added to the list of ready rays
4. When a ray is seen (gotten), it will be reset the next time it is touched to add points

Finally, the criterion being used currently is the number of points in a given ray. This was
for the sake of simplicity. When a full scan is received, all rays are considered ready, regardless
of the number of points in a ray (if the number of points is positive).

## Inputs / Outputs / API

See `autoware::perception::filters::ray_ground_classifier::RayAggregator` for more details.

## Inner-workings / Algorithms

In general, the `RayAggregator` is fairly straight forward.

However, there is one piece of complexity in its implementation:

Each ray internally has three states:
1. `NOT_READY`
2. `READY`
3. `RESET`

Where the last state is set when a ray is seen, and is used to reset the state of a ray upon
the next insertion to the ray.

# Security considerations

TBD by security expert.


# References / External links

See original Autoware implementation, where the `RayAggregator` is implicitly defined:
[CPFL's Autoware](https://github.com/CPFL/Autoware/blob/7b384fbdc97793d8bf7c1387d22c50e1fc9109e3/ros/src/sensing/filters/packages/points_preprocessor/nodes/ray_ground_filter/ray_ground_filter.cpp)

# Future extensions / Unimplemented parts

More exotic "scan ready" criterion may be added and/or evaluated, e.g.:
- Max point spacing
- Point variance
