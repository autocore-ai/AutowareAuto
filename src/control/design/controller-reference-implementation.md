Controller Reference Implementation {#controller-reference-implementation}
===================================

@tableofcontents


This document defines a reference implementation for the controller component.
A software component (i.e. a library, executable, set of executables, or software black box) that
satisfies at least the inputs, outputs, and behaviors defined in this document shall be able to
operate with any sufficiently compliant Autoware.Auto-derived autonomous driving stack.

# Introduction

A controller can be thought of as the bottom portion of the autonomous driving planning stack.
It generates commands for the vehicle interface such that the vehicle will follow a give
trajectory.

# Scope

This document lays out a minimal set of inputs, outputs and behaviors for a contrller.

In particular, this document defines message types for input and output, and the rationale behind
their definition. The behaviors are defined in terms of expected results, and the rationale is
similarly outlined.

An implementation compliant with these inputs, outputs and behaviors will be able to operate with
any sufficiently compliant Autoware.Auto-derived autonomous driving stack.

This document specifically does not outline any implementation details for a controller algorithm
(e.g. whether the current vehicle state is predicted using a motion model, what algorithms are
used, etc). These details are left to the algorithm developers.

Additional behaviors, and outputs can be used in a controller implementation. These should
be properly documented. If additional inputs are used, then the minimal behavior outlined in this
document should still be possible without the inclusion of the extra inputs.

# Reference Implementation

This section outlines the inputs, outputs and behaviors required for a minimal controller
implementation

## Inputs

The controller has three inputs:

- Transform messages
- Trajectory messages
- Vehicle kinematic state messages

### Transform Message

A standard `tf2_msgs/TFMessage` should be a secondary input.

**Rationale**: The controller may receive trajectories and vehicle kinematic states in different
coordinate frames. Having transforms available allows the two objects to be transformed into
compatible coordinate frames without having to inject helper nodes to do so.

### Trajectory Message

A trajectory message represents a sequence of kinematic states that a controller should try to
satisfy.

A trajectory message has the following form:

```
std_msgs/Header header
TrajectoryPoint[100] points
uint32 size
```
Where a trajectory point has the following form:

```
std_msgs/Duration time_from_start
float32 x
float32 y
Complex32 heading
float32 velocity_mps
float32 acceleration_mps2
float32 heading_rate_rps
```

And the heading field has the following form:

```
float32 real
float32 imag
```

A zero heading corresponds to the positive x direction in the given coordinate frame.

**Default Values**: All trajectory point and trajectory fields should default to 0

**Default Values**: All complex numbers should default to the pair (1, 0), which corresponds to a
zero heading.

**Extensions**: In the future, this message may include higher derivative information.

**Rationale**: A bounded number of points is provided since a new trajectory should be provided
at a regular rate. Bounded data structures also imply more deterministic communication times
due to having a bounded size.

**Rationale**: Trajectory points mirror, but are not the same as a standard JointTrajectory message.
This is because the standard message allows room for ambiguities whereas this message does not.

**Rationale**: Higher derivative information (e.g. acceleration, heading rate) is available to
provide a more expressive language for controllers and trajectory planners. If the information
is not needed, a value of zero is provided, which is semantically consistent for controllers and
motion planners

**Rationale**: Complex numbers are used to represent heading rather than an angle because it reduces
ambiguities between angles, and angle distances. In addition, it conveniently represents a
precomputed sine and cosine values, obviating the need for additional computations.

For more details on the message, see the formal message definition in the package
`autoware_auto_msgs`. While this message and it's rationales are duplicated here for convenience,
the message package should be taken as the source of truth.

### Vehicle Kinematic State Message

The vehicle kinematic state represents the vehicle's current kinematic state, and is used to
inform a control command that would cause the vehicle to follow the current trajectory.

The vehicle kinematic state message has the following form:

```
std_msgs/Header header
TrajectoryPoint state
geometry_msgs/Transform delta
```

The `TrajectoryPoint::time_from_start` field should also be filled with the time difference between
the current state and the previous state message.

The position fields of the `TrajectoryPoint` member should contain the position of the ego vehicle
with respect to a fixed frame, if available. If the frame is a dynamic frame (e.g. `base_link`),
then this position field should be taken as unavailable or invalid. The further kinematics of this
member should always be available, and populated according to the appropriate coordinate frame,
making the additional assumption that the coordinate frame is static.

The delta member is the transform of the message should represent the positional update of the
ego vehicle's coordinate frame relative to it's last position and orientation.

**Default Values**: The trajectory point should be appropriately default initialized

**Default Values**: The delta transform field should have zero translation and the unit quaternion
by default.

**Extensions**: State and transform uncertainty may be added as variances or covariance matrices.
This may be necessary for probabilistic algorithms (e.g. robust MPC)

**Rationale**: An underlying trajectory point is used because kinematic information may be needed
by controllers. Further, representing the vehicle state in a manner that mirrors the trajectory
fits with the semantic purpose of a controller: matching the current state to a state sequence

**Rationale**: A transform relative to the previous position and orientation of the ego vehicle is
provided because while absolute position may not always be available with accuracy, this relative
transform should be available with some degree of accuracy (e.g. via IMU information, odometry
information, etc.). The availability of this information allows for planning and tracking within a
local coordinate frame. For example, in a level 3 highway driving use case, a trajectory may be
rooted at the ego vehicle's position at some time stamp. The accumulation of these relative
transforms would allow for a controller to properly follow the trajectory.

**Rationale**: The kinematics are populated with a fixed frame assumption to simplify the updates
and usage of this field.

For more details on the message, see the formal message definition in the package
`autoware_auto_msgs`. While this message and it's rationales are duplicated here for convenience,
the message package should be taken as the source of truth.

## Outputs

The controller has two outputs:

- A primary vehicle control message
- A secondary controller diagnostic message

### Vehicle Control Command Message

A vehicle control command message has the following form:

```
builtin_interfaces/Time stamp
float32 long_accel_mps2
float32 front_wheel_angle_rad
float32 rear_wheel_angle_rad
```

This message type is intended to provide a proxy for a foot on the accelerator and brake pedal,
and a pair of hands on the steering wheel, while also providing a convenient representation
for controller developers.

These wheel angles are counter-clockwise positive, where 0 corresponds to a neutral, or
forward-facing orientation.

**Default Values**: The default value for all fields is 0

**Rationale**: A time stamp field is provided because a control command should be generated in
response to the input of a vehicle kinematic state. Retaining the time of the triggering data
should aid with diagnostics and traceability.

**Rationale**: A single acceleration field is provided because controllers typically plan a
longitudinal acceleration and turning angle for the rigid body. Braking is baked into this single
command for simplicity of controller development, and because it is generally not expected for both
the brake to be depressed and the accelerator at the same time.

**Rationale**: Wheel steering angles are provided since it allows the controller to only require a
simplified view of the vehicle platform, such as knowing the wheelbase length. Depending on the
implementation of a drive-by-wire interface, producing a given wheel angle may require knowledge
of the mapping between steering wheel angle to wheel angle.

**Rationale**: The rear wheel angle is also provided to enable support for rear drive vehicles,
such as forklifts.

**Rationale**: A frame is not provided since it is implied that it is fixed to the vehicle frame.

For more details on the message, see the formal message definition in the package
`autoware_auto_msgs`. While this message and it's rationales are duplicated here for convenience,
the message package should be taken as the source of truth.

### Controller Diagnostic Message

A controller diagnostic message is intended to report on the operational statistics of the
controller's algorithm. The message consists of the following fields:

```
DiagnosticHeader header
bool new_trajectory
string trajectory_source
string pose_source
float32 lateral_error_m
float32 longitudinal_error_m
float32 velocity_error_mps
float32 acceleration_error_mps2
float32 yaw_error_rad
float32 yaw_rate_error_rps
```

Where `DiagnosticHeader` has the following fields:

```
string name
builtin_interfaces/Time data_stamp
builtin_interfaces/Time computation_start
builtin_interfaces/Duration runtime
uint32 iterations
```

**Rationale**: Diagnostic information is helpful to detect if a fault occurred, or some incorrect
behavior leading up to a fault. It can also be used for infotainment applications.

For more details on the message, see the formal message definition in the package
`autoware_auto_msgs`. While this message and it's rationales are duplicated here for convenience,
the message package should be taken as the source of truth.

## Behaviors

At least the following behaviors are expected from a compliant controller implementation:

- Generate control commands
- Generate diagnostic messages
- Operate in a fixed coordinate frame
- Operate in a local coordinate frame

### Generate Control Commands

Given a new vehicle kinematic state, the controller should output a control command.

**Rationale**: This is the basic function of a controller

### Generate diagnostic messages

Given a new vehicle kinematic state, the controller should output a diagnostic message.

**Rationale**: This is the basic function of a controller, and is useful for logging and
diagnostics.

### Operate in a fixed coordinate frame

The controller should be able to receive a trajectory that is in a fixed frame (e.g. `map`) and
properly operate.

This means that it should be possible to use subsequent vehicle kinematic state messages to
determine the vehicle's current state (i.e. via transforming an absolute position, or accumulating
relative transforms), such that the underlying algorithm can generate a result.

**Rationale**: Depending on the set up of the autonomous driving stack, absolute position with
respect to a fixed coordinate frame may be the only source of position information available.
Depending on motion planning algorithms, the resulting trajectory may be rooted in an absolute
frame as well.

### Operate in a local coordinate frame

A controller should be able to receive a trajectory that is in a local coordinate frame at a
particular time stamp and properly operate.

This means that a controller should be able to receive a vehicle kinematic state, and either
transform it's pose relative to be relative to the trajectory's frame (which is rooted
at a particular time stamp), or compute its pose by accumulating relative transforms, and
then do computation appropriately.

**Rationale**: A motion planner may output trajectories in a local coordinate frame. In addition,
strong localization guarantees are not necessarily always available (e.g. highway driving in a
rural environment, or with a level 3 stack). In this context, it is still possible to generate
a local trajectory, and it should still be possible to follow this trajectory.

# References

- AutowareAuto#62

## Related Message Types

ROS:
- [JointTrajectory.msg](https://github.com/ros2/common_interfaces/blob/master/trajectory_msgs/msg/JointTrajectoryPoint.msg)
- [PointStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PointStamped.html)
- [PoseStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/PoseStamped.html)
- [TransformStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TransformStamped.html)
- [NavSatFix](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/NavSatFix.html)

Autoware:
- [AccelCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/AccelCmd.msg)
- [BrakeCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/BrakeCmd.msg)
- [ControlCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/ControlCommand.msg)
- [SteerCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/SteerCmd.msg)
- [VehicleCmd](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/VehicleCmd.msg)
- [Waypoint](https://github.com/CPFL/Autoware/blob/master/ros/src/msgs/autoware_msgs/msg/Waypoint.msg)

**Apollo**:
- [ControlCmd](https://github.com/ApolloAuto/apollo/blob/3d6f86e21a3c3ac43cf08423d4c3f92bc63ecac9/modules/control/proto/control_cmd.proto)

Configuration Messages
- [LonControllerConf](https://github.com/ApolloAuto/apollo/blob/3d6f86e21a3c3ac43cf08423d4c3f92bc63ecac9/modules/control/proto/lon_controller_conf.proto)
- [LatControllerConf](https://github.com/ApolloAuto/apollo/blob/e9156ced04a8f0e372c781d803fd43428ab6c497/modules/control/proto/lat_controller_conf.proto)
- [ControlConf](https://github.com/ApolloAuto/apollo/blob/51651b9105e55c14e65cc2bd349b479e55fffa36/modules/control/proto/control_conf.proto)
- [MPCControllerConf](https://github.com/ApolloAuto/apollo/blob/51651b9105e55c14e65cc2bd349b479e55fffa36/modules/control/proto/mpc_controller_conf.proto)

Algorithm Status Messages
- [PlanningStatus](https://github.com/ApolloAuto/apollo/blob/51651b9105e55c14e65cc2bd349b479e55fffa36/modules/planning/proto/planning_status.proto)
- [LocalizationStatus](https://github.com/ApolloAuto/apollo/blob/51651b9105e55c14e65cc2bd349b479e55fffa36/modules/localization/proto/localization_status.proto)
- [PlanningStats](https://github.com/ApolloAuto/apollo/blob/51651b9105e55c14e65cc2bd349b479e55fffa36/modules/planning/proto/planning_stats.proto)
