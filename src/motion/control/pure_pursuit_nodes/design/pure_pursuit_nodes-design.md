Pure pursuit nodes {#pure-pursuit-nodes}
=======================


# Purpose / Use cases

This package is a thin boiler-plate package that connects computation (pure_pursuit) and
communication (DDS/ROS 2).

This is necessary to make it easy to test the computation in isolation from communication.
Likewise, this package separates out the integration/communication tests for this package.


# Design

This is a thin wrapper around pure_pursuit.

The current pose is transformed using the corresponding `tf` topic, and then compute the control command.


## Assumptions / Known limits

This node assumes the global `tf` graph has a transformation between trajectory and pose frames.

See `apex_tf` for more details on the limitations of the transforms API.

See the
PurePursuit
design doc for more details.


## Inputs / Outputs / API

Inputs:
- `tf` topic: source frame is pose and target frame is trajectory (through `apex_tf`)
- trajectory topic (in the trajectory frame)
- vehicle pose topic (in the pose frame)
- pure_pursuit parameters

Outputs:
- vehicle motion controller topic
- diagnosis topic for the pure_pursuit

On top of this, the nodes can be configured either programmatically or via parameter file
on construction.

## Inner-workings / Algorithms

See `pure_pursuit` for more details.

# Error detection and handling

Most error handling occurs inside `rclcpp`, inside the `PurePursuit`
itself, or in the associated configuration class.

# Security considerations

These components inherit security considerations from `Node`, `BufferCore`, and
the core `PurePursuit` class.


# Future extensions / Unimplemented parts

- `tf` internal library for the pose transformation
- The way to subscribe `tf` topic should be deterministic.

# Related issues

- #2642: Initial implementation
- #2838: apex_app -> apex_auto
