Guidelines for Naming in Autoware.Auto {#autoware-common-naming-guidelines}
===========================================================================================================

# Summary

In order to facilitate development and integration of nodes in the Autoware.Auto stack, it is necessary to provide at least a basic set of guidelines for the naming of TF frames, nodes, packages, topics, and namespaces.
This allows developers to make reasonable assumptions about the topics and frames used in other nodes throughout the stack without having to comb through the code of those other nodes.
It also aides first-time users of Autoware.Auto in identifying the content of topics and frames based on their names and location in the namespacing heirarchy.

While these guidelines could not possibly cover every naming scenario, they provide general-purpose recommendations and a few specific examples.

---

[TOC]

# Package Names

REP-144 [1] defines the ROS standard for package names and packages in Autoware.Auto must adhere to this standard.

Additionally, it is a best practice to append `_nodes` suffix for packages which contain multiple ROS nodes.

# TF Frames

The high-level frames that are used across the stack are defined in the <a href="localization-design.html">localization-design</a> document.
In ROS, the `tf` architecture supports a single `tf_prefix` which is generally used to support multiple robots operating within the same `tf` tree [2].
Given that the Autoware.Auto stack is intended to be used on large, complex mobile robots with many frames, it is unlikely to be used on multiple robots operating simultaneously within the same architecture.
Therefore, the stack is not required to support the `tf_prefix` convention [2].

# Namespaces

ROS allows for topics, parameters, and nodes to be namespaced which provides the following benefits:

- Multiple instances of the same node type will not cause naming clashes
- Topics published by a node can be automatically namespaced with the node's namespace providing a meaningful and easily-visible connection
- Keeps from cluttering the root namespace
- Helps to maintain separation-of-concerns

When a node is run in a namespace, all topics which that node publishes are given that same namespace.
All nodes in the Autoware.Auto stack must support namespaces by avoiding practices such as publishing topics in the global namespace as described in REP-135 [4].

## Node and Topic Namespace Guidelines

It is recommended that nodes within the Autoware.Auto stack which do not provide raw sensor data be namespaced based on a category into which the function of that node falls.
This list is likely to be controversial based on how an individual views the categorization of a given node but this list is not intended to be exhaustive nor set in stone.
A general rule-of-thumb is that a namespace category should only contain one instance of a node which performs a major function in that category.
Some recommended namespace categories are:

- `control`
- `localization`
- `navigation`
- `perception`

"General-purpose" or "utility" nodes should fall into the namespace associated with the context in which they are being used.
For example, a node which transforms object data from one frame to another is not specific to a given stack funcion but should be placed in `planning` if it is being used in the plannig architecture or `perception` if it is being used in the perception architecture.

In general, topics should be namespaced based on the function of the node which produces them and not the node (or nodes) which consume them.

## Special Case: Sensors and Sensor Drivers

Since each sensor on a mobile robot must have its own `tf` frame which must be globally unique, it makes sense to use the same name for both the `tf` frame and the topic/node namespace.
The following recommendations are provided and apply to both the `tf` frame name and topic/node namespace.

The recommended convention for all sensors where multiple sensors of the same type are present is `<sensor_type>_<direction>`.

If only a single instance of a given sensor type is used on a robot, the `_<direction>` can be omitted.
If a single node publishes the data for multiple sensors, the node does not need to have a namespace but the sensor frames and topic names should still follow these guidelines.

If a single node or driver provides data for multiple sensors of the same type prior to any post-processing, a plural form of the sensor type is recommended (e.g. `lidars`, `radars`, etc.).

### Sensor Type Prefix

In the above naming convention, `<sensor_type>` represents a generic name for the type of sensor.
This generic name should be manufacturer- and model-agnostic and describe the general "type" of the sensor rather than the sensor's function.
Some examples include:

- `lidar` (as opposed to `velodyne` or `os-1`)
- `radar` (as opposed to `delphi` or `esr`)
- `camera` (as opposed to `mako` or `rgb`)

#### GNSS/GPS/GLONASS/Galileo/BeiDou

Since GPS, GLONASS, Galileo, and BeiDou are specific types of global positioning systems representing the use of only a single constellation of satellites and GNSS is a superset which can contain any combination of these [3], the recommended `<sensor_type>` is `gnss`.

### Direction Suffix

In the above naming convention, `<direction>` represents one or more of the following body-relative directions which are determined while standing behind the robot, facing toward the front of the robot (X-forward in ROS RHR convention).
If multiple body-relative directions are used, they should be seperated by underscores.
The order of multiple body-relative directions is recommended to be X, Y, Z in the ROS RHR convention to avoid confusion.

- left
- right
- front
- rear
- top
- bottom
- middle

### Sensor Node Namespace/TF Frame Examples

- `camera_front_left`
- `gnss`
- `imu`
- `lidar_front`
- `radar_rear_middle`
- `ultrasonics` (if data from multiple sensors are aggregated)

### Sensor Data Topic Names

For data which are published in a message type that represents the full, unprocessed sensor output, the recommended form is `<data_type>_raw`.

For data which are published in a message type that represents raw sensor data but the topic contain a subset of the original sensor's full output, it is recommended to use the form `<data_type>_filtered`.
If multiple filtering stages exist for a given data stream, applying another suffix before `_filtered` can help differentiate between the stage outputs (e.g.  `<data_type>_downsample_filtered`).

For data which are published in a message type that represents raw sensor data but the topic contains data which have been combined from multiple sensors of the same type, it is recommended to use the form `<data_type>_fused`.

#### Examples

- `camera_rear/image_raw`
- `gnss/nmea_raw`
- `imu/imu_raw`
- `lidar_front/points_filtered`
- `radars/returns_fused`

# References
- [1] https://www.ros.org/reps/rep-0144.html
- [2] http://wiki.ros.org/geometry/CoordinateFrameConventions#geometry.2BAC8-CoordinateFrameConventions.2BAC8-Naming.tf_prefix
- [3] https://en.wikipedia.org/wiki/Satellite_navigation
- [4] https://www.ros.org/reps/rep-0135.html
