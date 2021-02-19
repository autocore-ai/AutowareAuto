off_map_obstacles_filter {#off-map-obstacles-filter-package-design}
===========

This is the design document for the `off_map_obstacles_filter` package.


# Purpose / Use cases
Provide the core functionality of the `off_map_obstacles_filter_nodes` package – see its design doc for background.


# Design
The core functionality is provided in the form of a member function that deletes bounding boxes that do not have substantial overlap with the map.


## Assumptions / Known limits
A fundamental assumption is that the `base_link` and `map` frames' z axes are collinear. Another assumption is that the bounding boxes are only rotated by yaw, not roll or pitch.


## Inputs / Outputs / API
See the API documentation.


## Inner-workings / Algorithms
The algorithm takes the bounding boxes, transforms them to the map frame, and converts them to `lanelet::Polygon2d`s.
Each polygon then fetches potentially-overlapping primitives from the map using a bounding box intersection test.
Each of those candidates for overlap is tested with an exact algorithm. `lanelet2` integrates well with `boost::geometry`, so that is used to calculate the overlap percentage.


## Error detection and handling
Error detection isn't really done.


# References / External links
See the [Lanelet2 geometry primer](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/GeometryPrimer.md).


# Future extensions / Unimplemented parts
In the future, we should remove this package entirely.


# Related issues
Issue #840.
