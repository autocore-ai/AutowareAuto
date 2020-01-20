Localization Design {#localization-design}
=========================================

# High Summary

The Autoware.Auto localization stack should be compliant with REP105[3], with the following frames
defined:
- `/earth`
- `/map`
- `/odom`
- `/base_link`
- Additional coordinate frames may be attached to the `/base_link` frame to represent sensor frames

A localization stack should have the following components:
- Map manager (outputs `/earth`-`/map` transform)
- Transform manager (outputs full chain of transforms)
- Absolute Localizer (e.g. GPS, outputs `/earth`-`/base_link` transform)
- Relative Localizer (e.g. NDT Matching, outputs `/map`-`/base_link` transform)
- Local Localizer (e.g. visual odometry, IMU, outputs `/odom`-`/base_link` transform)

Mapping is considered to be a separate, but closely related concern to localization.

In addition, downstream components should make use of the following component, which understands the
semantics laid out in this document:
- LocalBufferCore (Updates `/odom`-`/base_link` transforms, permits `/base_link`-`/base_link`
transforms across time)


With the following interfaces:
- Reference map (input to relative localizer):
[PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)
- Relative localization status:
[DiagnosticStatus](https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/DiagnosticStatus.msga)
- Output of all localization components (for consistency):
[TFMessage](https://github.com/ros2/geometry2/blob/ros2/tf2_msgs/msg/TFMessage.msg)

Depending on the use case, some subset of components may be needed.

---

[TOC]


# Introduction

Localization is an important part of any higher-level autonomous driving stack. While minimal level
3 functionality is possible with purely reactive control, any higher levels of autonomy, or more
advanced level 3 functionality requires planning. In order to use a plan, an agent must know where
it is with respect to the plan, during the execution of the plan. This is what localization
provides.

In addition, localization provides the underlying functionality needed to solve harder problems in
autonomous driving, such as those that require a map. These problems include handling complex road
structures, such as intersections, and the global routing problem.

# Use Cases

At a high level, a localization algorithm must do one thing: given an observation, the algorithm
must provide the transform of the observation's coordinate frame with respect to some reference
coordinate frame.

The transform is the full 6 degree of freedom transform used to represent an arbitrary rigid-
body translation and rotation in 3 dimensional Cartesian space.

In general, we can subdivide the use case into three kinds of localization, with different strengths
or guarantees:
- Absolute localization
- Relative localization
- Odometry (i.e. visual odometry)

This relative localization use case can then be applied to, or broken down into various specific
use cases:
- Mapping/SLaM, i.e. localization with respect to a dynamic map
- Localization with respect to an apriori map, i.e. localization with respect to a static map

In addition, various input modalities can be considered, including:
- LiDAR
- Camera
- GPS/IMU/INSS

Usages of the output or result of localization can also be broadly considered:
- Global algorithms:
  - Behavior planning
  - Routing/global planning
- Local algorithms:
  - Motion control
  - Motion planning
  - Object tracking/occupancy mapping
  - Scene understanding (e.g. lane detection)
  - Motion estimation (e.g. sensor fusion)
- Dead reckoning

Finally, the concrete use case of a long-distance travel (i.e. cross-country) in a caravan can be
considered.

## High-level use cases

Each of the high level use cases are described below.

### Absolute localization

Absolute localization involves providing the vehicle pose with respect to an absolute inertial frame
that does not change, for example, providing the vehicle pose in LLA (or ECEF, preferred)
coordinates, which is with respect to the center of the earth.

In this context, GPS can provide absolute localization. Absolute localization may also be possible
by using the ego position relative to fixed, unique features, such as a unique sign on the road.

Such localization algorithms would typically provide an estimated pose in LLA coordinates with at
least double precision.

### Relative localization

Relative localization involves computing a transform of an observation in the ego coordinate frame
to a transform of a local map's coordinate frame. The local map is typically in ENU coordinates,
with some ECEF reference point.

Most localization algorithms used in autonomous driving fall under this category.

These algorithms often solve nonconvex optimization problems, implying the solutions are local
minima. This implies that these algorithms generally require a good initial guess, which can be
provided by an absolute localization component, or warm-started from a previous solution.

### Odometry

Odometry involves computing a transform of the ego vehicle with respect to local features,
such as lanes, road boundaries, and signs.

Algorithms in this space overlap with SLaM algorithms, and other feature extraction methods.

This localization modality would typically be used in mapless driving, such as in a level-3 highway
autopilot scenario, or in the case where higher forms of localization are deprived, such as when
traversing a GPS dead zone or a space where the map quality is insufficient for good relative
localization (i.e. a tunnel).

## Map types

A fundamental distinction in the mode of operation of a localizer is if the reference map is static
or dynamic.

### Static Map

If a static map is provided, then the localization component is expected to produce some transform
for the observation frame relative to the frame of the map.

Generally speaking, a map should be in a ENU coordinate frame to be compatible with simple Cartesian
reasoning and planning.

In this context, a map's origin should have some ECEF coordinate associated with it. A map should
likely be reasonably bounded in size (e.g. < 83 km[3]) so as to avoid error induced due to the
curvature of the planet.

Because of this, it is likely necessary to geofence or implement a mechanism that allows the
localizer to switch static maps. This switching should be done while the vehicle is still
within the bounds of the current map. The subsequent map should overlap with the current map,
and any objects referenced in the map frame or any derivative of such should have an updated pose.

In general, the handling, tiling, switching, and representation of such static maps are
implementation specific.

### Dynamic Map

Generally speaking, a map for a localizer is dynamic if mapping is intended to be done, either as
a core function (i.e. to later generate a static map), or as a part of a larger use case (i.e.
SLaM: Simultaneous Localization and Mapping).

In this context, the observations must generally be stored independently. In the presence of new
observations, the aggregate map may change, such as due to the effects of bundle adjustment
or pose graph optimization.

---

In general, the representation of a map should have no influence on the external behavior of a
localization component.

## Input use cases / Sensors

Localization can be achieved using a variety of sensors. Some of the use cases and considerations
therein are considered in this section.

In general, localization algorithms work by extracting distinctive features from both the current
observation and the reference, and determining the transform which best aligns the observed features
with reference features.

### LiDAR

A number of local registration algorithms exist, which operate off of point, line, plane, or
distributional features.

Common algorithms in this field include ICP and NDT.

Global registration algorithms also exist.

### RADAR

Many techniques that work for LiDAR can also work with RADAR, as both are active sensors which
produce return in 3D Cartesian space.

An example of an algorithm which works for both LiDAR and RADAR is ICP [1].

### Camera

Localization via cameras work similarly to LiDAR or RADAR in that generally speaking features are
extracted from the observed image, and matched against reference features.

Camera localization techniques differ from 3D localization techniques in that depth must be
inferred, typically by solving an optimization problem.


### INSS (GPS, IMU)

GPS as a localization modality differs from the other sensing modalities in that the reference
for GPS takes the form of GPS satellites.

GPS (and INSS) sensors produce absolute position estimates, typically in the Earth-relative LLA
coordinate frame. To use these results, in a way that is consistent with other localization
implementations these LLA coordinates should be transformed to the local ENU frame via a reference
ECEF point.

IMUs by contrast provide linear acceleration and rotational velocity observations. These
observations generally arrive very frequently (e.g. at 1 kHz), and can be accumulated using a
motion model (either naive or more informed) over a period of time to produce a position update.
These position updates can be combined with a localization source that can provide a position
with respect to the reference frame.

---

In general, the handling of different input or sensor modalities is implementation-specific.
A compliant implementation should provide a 6 DoF transform between the ego frame and some
fixed map or reference frame.

## Output use cases

The output of a transform of the ego frame with respect to a fixed or reference frame can be
used by many algorithms in the autonomous driving stack.

### Global algorithms

Global algorithms operate with respect to a coordinate frame that is fixed in the world. These
algorithms general involve some longer range planning.

#### Behavior Planning

A behavior planner generally sets up a motion planning problem based on the current pose, the local
road network, and obstacles within or near the region of interest.

A behavior planner may not need a full fidelity representation of the vehicle's state, but it will
need transforms such that all inputs to this component are in compatible coordinate frames.

In general, a behavior planner will operate in the `/map` coordinate frame.

#### Global planning

A global planner computes a sequence of lanelet sets which can bring the ego vehicle from its
current pose in the world to a target pose.

A global planner would only need a coarse representation of the vehicle's state (i.e. position,
heading, velocity). The localization component would need to provide a pose in a coordinate
frame that is compatible with the world frame.

In general, a global planner will operate in the `/earth` or `/map` frame, depending on the use
case.

### Local algorithms

Local algorithms generally operate in a frame local to the ego vehicle. These algorithms typically
directly use observations and operate in the `/odom` frame. If no source of odometry is available,
then these algorithms can alternately operate in the `/map` coordinate frame.

#### Motion estimation

The pose, or transform of the ego vehicle with respect to a fixed (inertial) frame can be
accumulated over time to produce estimates about the dynamics of the dynamics of the vehicle.
A simple example of this might involve numerically differentiating the change in position over
time to derive the velocities of the vehicle.

More advanced forms of motion estimation can be used, such as those using motion models and
state estimators.

Motion or state estimation should be kept separate from localization, and can take in the produced
transforms as an input, and generate a full kinematic or dynamic state of the vehicle as an output.
State estimators may benefit from a probabilistic handling of the observed pose or transform.
In such a case, it may be beneficial to represent Gaussian noise about the transform and Bingham[2]
noise about the rotational quaternion, resulting in an additional 9 + 16 = 25 doubles in the message
representation.

Such dynamics estimates can be used by the planning and control components of the autonomous driving
stack.

#### Motion Control

Given a trajectory, and a current state, a motion controller produces a control command which
bounds the error of subsequent states with respect to the reference trajectory.

At each iteration, a state which denotes where the vehicle is with respect to the reference
trajectory must be provided. The results of localization can be used to populate the state,
or transform the state into a coordinate frame that is compatible with the reference trajectory.

In this context, robust control may benefit from the addition of uncertainty parameters.

#### Motion Planning

Given a current state, a target state, obstacles, and a drivable space, a motion planner generates
a trajectory that is compatible with the current state, stays within the drivable space, does not
collide with obstacles, and makes progress towards the target state.

The results of localization may be used to implicitly generate the current state, and it may be used
to ensure that all of the inputs to a motion planner are in a compatible coordinate frame.

Similarly, robust planning algorithms may benefit from the addition of uncertainty parameters.

#### Object Tracking / Occupancy Mapping / Scene Understanding

Object tracking, occupancy mapping, and scene understanding algorithms generally follow similar
patterns. These algorithms involve receiving observations via (processed) sensor input, and
use these observations to update a posterior state estimate of some world property.

Fundamentally, this involves making sure the state of the algorithm is in a consistent coordinate
frame as the observation. As an example, this would involve ensuring that a transform exists
between the ego, or base link coordinate frame which objects are observed in, and some inertial
coordinate frame, for example a map frame, which object tracks are stored in.

### Dead reckoning

Another common use case is dead reckoning when stronger localization signals are unavailable. For
example, if a RTK GPS is used as the primary form of localization, some dead reckoning may be needed
when the ego vehicle is in a GPS-deprived location, such as in a tunnel.

Some ways to handle these kinds of situations include using odometry, IMU, or other local
observations, such as lane markings.

By and large, dead reckoning can be considered a concern separate from localization, as it cannot
provide absolute position, and only relative position. Dead reckoning can be used as a way to
combine sensing modalities within a localization implementation to improve update rates (e.g.
using GPS with IMU). Alternatively, dead reckoning can be implemented as a part of motion estimation
(i.e. as a part of the temporal prediction step).


## Concrete Use Case

The concrete use case of doing long-distance travel in a caravan is considered. This would involve
traveling at high speeds for at least three hours.

Depending on the details of the hardware setup, all three forms of localization may be needed.

If a high-precision RTK GPS is used, and GPS is available at all times during the trip, then
no additional mechanisms will be needed besides a mechanism to translate absolute coordinates
to a local frame, in addition to periodic updates to the local frame (i.e. map switching),
as planning generally occurs in the local frame.

If some GPS-deprived locales will be traversed, then a form of odometry or relative localization
must be used in the GPS-deprived regions. This may involve maintaining a reference map which can be
localized against, or maintaining local updates that can support local planning and tracking
processes.

If the primary localization modality used is relative localization, then a form of absolute
localization must be used upon startup to determine which is the most relevant reference map,
and an initial estimate of where the ego is in this reference map. In addition, map switching
must be handled in a way that does not disrupt subordinate processes, such as tracking and planning.
The same general concerns apply when traversing GPS-deprived regions as in map-deprived regions.

# Requirements

A localization implementation must satisfy a few requirements in order to be compatible with the
aforementioned use cases:
- Provide a rigid body transform between the ego frame and a specified inertial frame
- Provide a transform compatible with the `/earth` frame
- A mechanism for managing reference maps must be provided if relative localization is used
- A relative localizer should have a mechanism that permits initialization either from a startup
or reference-deprived state, from either an absolute localization, or a local source
- An implementation of a localization stack should be compliant with REP105[3]
- The `/odom`-`/base_link` transform must not be allowed to grow in magnitude indefinitely.

## Provide a transform between ego and inertial frame

The core functionality of localization algorithms satisfies this requirement. This requirement
fulfills the following use cases:
- Global planning (if the inertial frame is compatible with the world frame of the road network)
- Behavior planning
- Motion planning (local or relative localization can be used depending on target frames)
- Motion control (local or relative localization can be used depending on target frames)
- Motion estimation
- Object tracking (local or relative localization can be used depending on target frames)

If a method which provides absolute localization is used, such as GPS, then some mechanism must be
provided which translates this absolute position into a local frame.

This requirement may have additional subclauses, such as those relating to properties of the result
or performance of the mechanism, for example:
- Bounded error
- Bounded runtime

**Rationale**: Planning, tracking and motion estimation generally take place in Cartesian/ENU/local
coordinate frames

## Provide a transform compatible with the `/earth` frame

For the long-term planning use case, and the use cases which involve the use of a HD map, then an
absolute transform with respect to the `/earth` frame must be available. As such, some mechanism
must be provided which connects the local inertial frame with the absolute `/earth` frame.

**Rationale**: Some useful features are provided in the `/earth` frame, either via LLA or ECEF
coordinates.

## Updating Local inertial frame

Planning, tracking, and motion estimation generally take place in a local Cartesian/ENU coordinate
frame. In addition, these algorithms typically make a planar motion assumption, as road participants
generally stay affixed to the ground. Given that the Earth is round, as 2D coordinates become more
distant from the origin of the local coordinate frame, error is accumulated.

As such, it is important that the region of influence of the local coordinate frame is bounded and
as the ego vehicle moves near the bounds of the local coordinate frame, it should be switched to a
new local coordinate frame.

This is necessary to prevent unbounded errors from being introduced to local processes (i.e.
planning, tracking, motion estimation).

**Rationale**: The earth is round, and many algorithms make planar assumptions. Map switching is
needed to control the influx of approximation errors

**Rationale**: Memory is not unbounded, so it is unlikely that a reference map for the whole world
can be stored

## Reference localizer initialization

To ensure optimal operation of a reference localizer, initial pose estimates should be provided.

**Rationale**: Most relative localization algorithms need a reasonable initial guess to converge to
a good estimate.

## REP105 Compliance

REP105[3] is a standard specifying coordinate frames for a robot. Various concerns are baked into
the choice and design of the coordinate frames, including:
- Map switching
- Localization drift/correction
- Map coordinate frame and initialization conventions

**Rationale**: Compliance to this standard will make this localization stack immediately
understandable to external experts

**Rationale**: Adherence to this standard allows implementations not purpose-build for the
autonomous driving use-case to be used in this localization stack

**Rationale**: This standard specifies behaviors which can account for various failure cases and
encodes community expertise and expectations

## /odom-/base_link transform must be bounded

Local algorithms involve transforming constructs with respect to different times and space. An
implicit assumption is that these transforms will be smooth across time. As such, local algorithms
must operate with respect to the `/odom` coordinate frame, which has the built-in assumption of
having smooth evolution. This typically assumes that the `/odom` frame origin is fixed in space
without moving.

If a vehicle is travelling for long distances, then the magnitude of the `/odom`-`base_link`
transform will grow in magnitude, possibly to the point where floating point precision becomes an
issue[3]. As such, we require a mechanism to ensure that the magnitude of this transform remains
reasonably small, to avoid errors from floating point arithmetic at large magnitudes.

**Rationale**: Floating point arithmetic incurs errors, which are proportional to the magnitude of
the values involved in the arithmetic. These errors should be minimized, thus implying that the
magnitude of the transform should be bounded or minimized.

# Mechanisms

The primary mechanism to satisfy this requirement is the selection of an appropriate localization
algorithm.

- Depending on the use case, some subset of localization algorithms must be provided, as a mechanism
to satisfy the basic requirement
- If absolute localization is used, this signal must be converted to the local frame via a software
component to satisfy the basic requirement
- A software component which switches maps must be provided to satisfy the update of the inertial
frame
- A mechanism must be provided which can provide an initial guess for relative localization
algorithms
- A mechanism to batch update the frame graph such that the `/odom`-`/base_link` transform can be
kept small, and ensure smoothness, and global consistency is needed

These mechanisms are partially instantiated in the general localization architecture as described
below.

# Design

In order to support the above stated use cases and derived requirements, a decomposition of the
localization stack is proposed below.

## Components

In the most abstract case, we define the following components in a localization stack. Depending on
the use case and capabilities of various components, only some subset of the components may be
needed.

- Map manager
- TF manager
- Absolute Localizer (e.g. GPS)
- Relative Localizer (e.g. NDT Matching)
- Odometry

To support the control of magnitude of the `/odom`-`/base_link` transform, an additional mechanism,
embedded in downstream algorithms, LocalBufferCore, is required.

### Map Manager

A map manager maintains control over the local inertial frame. It is the responsibility of the map
manager to provide reference maps to relative localizers for the local inertial frame, and the
appropriate ECEF reference point for the local coordinate frame. This component must also handle the
update of local maps and appropriate communication of these changes to the relative localizer
and other components which may be affected.

**Input**: The full transform tree at a given time step (i.e. a message containing the following
transforms at one or more time steps: `/earth`-`/map`, `/map`-`/odom`, `/odom`-`/base_link`)

**Output**: Reference map (localization)

**Output**: A paired `/earth`-`/map` and `/map`-`/odom` transform

**Output**: Other map features (i.e. planning)

**Behavior**: This component should manage the switching of maps. When the vehicle approaches the
limit of a map, this map should load a new map from a store, and make the features of the map
available to the larger system. The limit of a map can be defined as being within 10 seconds away
from the physical limits of the map, or being within 10 seconds
away from the last road sequence which can keep the vehicle within the bounds of the reference map.
If the use case does not require map switching, then map switching may not be needed, and only
simple loading and publishing of data.

**Behavior**: If a solved transform is near the boundaries of a reference map, a warning should be
communicated to the larger stack. "Near" can be defined as being within 10 seconds away from the
physical limits of the map (in which case a system may come to a stop), or being within 10 seconds
away from the last road sequence which can keep the vehicle within the bounds of the reference map
(in which case a system may reroute the vehicle in the absence of a new map).

**Behavior**: The `/earth`-`/base_link` transform should be used to determine when map switching
should occur

**Behavior**: When the `/map` frame updates, the inverse of the `/earth`-`/map` applied transform
should be applied to the latest `/map`-`/odom` transform and published along with the update

**Rationale**: Using the full transform tree update as the input simplifies the initialization of
this component with respect to various use cases

**Rationale**: A paired update is needed to maintain consistency of the `/base_link` coordinate
frame in the absolute or global frame.

**Rationale**: Loss of localization is a critical error. This situation should be proactively
avoided by properly notifying the larger system.

### TF Manager

Each of the localization modalities provides the pose of the ego vehicle with respect to their
reference coordinate frame. These transforms are generally not compatible with the transform tree
as defined.

As such, a component is necessary which can combine all the localization modalities and ensure
they are outputted in a way that is consistent with the defined transform tree.

This component coordinates the various parts of a localization stack, and represents the primary
output of the localization stack and its interface with the rest of the autonomous driving system.

**Input**: Absolute localization (`/earth`-`/base_link`)

**Input**: Relative localization (`/map`-`/base_link`)

**Input**: Odometry (`/odom`-`/base_link`)

**Input**: Map update (`/earth`-`/map`)

**Output**: The full transform tree at a given time step (i.e. a message containing the following
transforms at one or more time steps: `/earth`-`/map`, `/map`-`/odom`, `/odom`-`/base_link`)
- This message may also occasionally contain a `/odom`-`/odom` transform to signal an update of
the position of the `/odom` frame and child transforms

**Behavior**: The TF manager can only be considered initialized when it has received a
`/earth`-`/map` transform. This may involve some preliminary output from this component

**Behavior**: When an odometry input is received, this transform is added to the latest
`/odom`-`/base_link` transform, the resulting transform is added to the frame graph, and to a
message containing cached map and odometry frame transforms and published

**Behavior**: When a relative localization input is received, the odometry transform at the
appropriate time stamp is subtracted from it to provide a `/map`-`/odom` transform

**Behavior**: When an absolute localization input is received, the map and odometry components of
the transform can be subtracted to provide a `/map`-`/odom` transform

**Behavior**: When a map update input is received, the new transform message should be input into
the output message, and the previous map transform should be included with a time just before the
new transform

**Behavior**: If absolute and relative localization inputs are available for the same time frame,
the update to the `/map`-`/odom` frame is implementation defined. These implementations may include:
- Fusion of transforms via sensor fusion algorithms
- Preferring the more precise transform
- Preferring the more transform that arrives sooner

**Behavior**: Periodically (i.e. every N seconds, or every N odometry updates), the TF manager
should signal an update to the `/map`-`/base_link` frames by prepending an appropriate
`/odom`-`/odom` transform to the beginning of the output message. The remaining output may either:
- Be a nominal update for the given time instant
- Contain a full transform tree (with `/odom`-`/base_link` history)

**Behavior**: On startup, the every transform is initialized to be the identity transform

**Rationale**: This component is necessary to isolate the concerns of each individual localization
modality, thus simplifying their implementation.

**Rationale**: All transforms are initialized to the identity transform to
simplify initialization of other components. When the appropriate map is initialized, a step
transform can be induced to update the `/earth`-`/map` transforms without affecting the
`/odom`-`/base_link` transform, ensuring continuity

**Rationale**: Handling multiple localization modalities is use-case specific. This document only
outlines the basic behavior of updating the appropriate coordinate frames

**Rationale**: An update of the odometry frame is necessary to ensure the transforms are bounded in
magnitude and to minimize errors in floating point arithmetic. Inverse updates of the historical
transforms are needed to ensure global consistency of the coordinate frames. Updates may be of
different forms to accommodate a communication-computation trade-off.

### Absolute Localizer

An absolute localizer (e.g. GPS driver) provides the current pose of the ego in an absolute frame.

**Input**: N/A (Implementation defined, e.g. sensor input)

**Output**: The transform `/earth`-`/base_link`, where rotation is such that the z axis is pointing
away from the earth, and the x-axis is pointing to the east

**Rationale**: This is an independent sensing modality

\note If a GPS driver is used which produces the
[NavSatFix](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/NavSatFix.msg)
message, an additional component may be needed to convert the point into an appropriate ECEF
transform.

### Relative Localizer

Relative localization algorithms involve producing a transform to best *match* or *register*
observations with respect to a ground truth. These algorithms generally require a good initial guess
in addition to the ground truth reference. Depending on the architectural assumptions, and the
parameters of the given use case, motion constraints may also be encoded into the implementation.

**Input**: N/A (Implementation defined, some sensor input)

**Input**: A reference map

**Input**: The transform tree, within which the `/map`-`/base_link` transform is used for
initialization

**Output**: Provides 6 DoF rigid body transform (translation, rotation) with respect to a reference
map (A transform between `/map`-`/base_link`)

**Output**: Some form of diagnostic should be provided, specifying the state of the algorithm

**Behavior**: The reference map must be validated to ensure all necessary features are present

**Behavior**: If a transform between `/map`-`/base_link` is available at the time stamp of the
sensor input, this transform should be used to initialize the algorithm.

**Behavior**: If no initial pose estimate is available for the time stamp of the sensor input,
then the behavior is implementation defined.

**Rationale**: A number of methods can be used for cold start initialization, including zero-
initialization, or reading a file written upon shutdown. The appropriateness of these methods
are use-case dependent, and thus cannot be pre-specified for all implementations

**Rationale**: Similarly, a number of methods can be used for extrapolation, such as using
the last transform, or predicting the current pose using a motion model. The appropriateness of
any method depends on the use case.

The inputs to a (relative/local) localization algorithm are implementation-defined. These inputs
may include, but are not limited to:
- LiDAR point clouds (raw or downsampled)
- RaDAR blobs
- Camera images
- IMU readings
- Odometry readings

### Odometry

Odometry works similarly to relative localization. In this case, the algorithm provides
transforms with respect to previous observations. Algorithms that fulfill this component's
behavior include (visual/LiDAR) odometry algorithms, and SLaM algorithms.

**Input**: N/A (Implementation defined, some sensor input)

**Output**: Provides 6 DoF rigid body transform (translation, rotation) with respect to a previous
observation, characterized as a transform `/ego`-`/base_link`

### LocalBufferCore

In order to keep the `/odom`-`/base_link` transforms bounded in size, batch updates of transforms
are needed. These updates involve updating the `/map`-`/odom` transform with a given transform, and
updating the history of `/odom`-`/base_link` transforms with the inverse of the aforementioned
transform to ensure global consistency is maintained.

This component would be a wrapper around `tf2::BufferCore`.

The following API is proposed:

```cpp
class LocalBufferCore : protected tf2::BufferCore
{
public:
  // Handles whole frame graph update when /odom-/odom message is received; also handles queueing of
  // /odom-/base_link transforms to support this
  void addTransforms(const TFMessage & msgs);
  // Handles transforms from /base_link (past) to /base_link (present), or vice versa
  bool canTransform(const std::string & frame_id, time from, time to);
  // Handles transforms from /base_link (past) to /base_link (present), or vice versa
  TransformStamped lookupTransform(const std::string & frame_id, time from, time to);
  using BufferCore::canTransform;
  using BufferCore::lookupTransform;
};  // class tf2::BufferCore
```
**Rationale**: This component is needed to ensure updates to the `/odom` frame are synchronized
across a distributed system

**Rationale**: This component allows local algorithms to operate in the `/base_link` frame, and
update their internal state to be consistent with observations if need be

**Rationale**: If a local algorithm is computing transforms across time in the `/base_link` frame,
then an accumulated transform should be the same between two time points before and after an
`/odom`-`/odom` update

# Interface definitions

Concrete interface definitions are proposed for the above design.

## Output

All components output the standard
[TFMessage](https://github.com/ros2/geometry2/blob/ros2/tf2_msgs/msg/TFMessage.msg).

**Rationale**: Some components may output one or more transform

**Rationale**: This is a standard message

**Rationale**: All components should use this to maintain type consistency

**Rationale**: For components which generally output one transform, the use of this type allows
for extensibility or batching, as appropriate

### Extensions

If the use of uncertainty semantics is needed by algorithm developers, then the following message is
proposed:

```
# TransformWithCovariance
geometry_msgs/TransformStamped transform

float64[9] translation_covariance
float64[16] rotation_covariance
uint8 position_covariance_type
uint8 rotation_covariance_type

uint8 COVARIANCE_TYPE_UNKNOWN = 0
uint8 COVARIANCE_TYPE_APPROXIMATED = 1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2
uint8 COVARIANCE_TYPE_KNOWN = 3
```

```
# TFCovarianceMessage

TransformWtihCovariance transforms[]
```

Where the `rotation_covariance` can have Bingham[2] semantics.

**Rationale**: A transform message is a standard message which denotes a rigid body transform in
standard Cartesian space

**Rationale**: Uncertainty is added to mirror that in
[NavSatFix](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/NavSatFix.msg).
This additional information may be of use to probabilistic or robust algorithms used downstream,
such as motion estimation or planning algorithms

\note The use of these custom messages are not recommended at this time due to overhead and breaking
with standard messages

## Reference map representation

The
[PointCloud2](https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/PointCloud2.msg)
is proposed as a means of reference map representation.

**Rationale**: It's a standard message type

**Rationale**: It's extensible to arbitrary representations and layouts that might be implementation
specific

## Relative localization diagnostic

The health status of a relative localization algorithm may be reported with the
[DiagnosticStatus](https://github.com/ros2/common_interfaces/blob/master/diagnostic_msgs/msg/DiagnosticStatus.msga)
message.

**Rationale**: This is a standard message type

**Rationale**: The semantics and expected result of a message of this form is different from that
of a DiagnosticHeader, which has semantics of reporting general algorithm performance

# Example Architectures

To demonstrate validity of the design, and provide guidance for implementers, a few example use
cases and architectures are proposed.

## GPS-Only

A GPS-only stack would consist of the following components:
- Absolute localizer (GPS driver)
- Map manager (only periodically updates local reference frame)
- TF manager

Because there is no odometry, local algorithms using this output would likely have to be set up to
operate with respect to the `/map` frame to ensure consistency.

## NDT-Based

At a bare minimum, an NDT-based (or any other relative localizer) stack would need the following:
- Relative localizer (NDT matching)
- Map manager
- TF manager

The map manager can simply produce a single map, if the map appropriately covers the service area.
Alternatively, if the relative localizer is appropriately initialized with an appropriate map,
then the map manager can know which map to switch to based on positions with respect to the
reference map.

If no other form of initialization is available, or specified by the use case, then the following
additional components are needed to ensure the stack can properly initialize:
- Absolute localizer

Similarly, in this setting, because there is no odometry, local algorithms using this output would
likely have to be set up to operate with respect to the `/map` frame to ensure consistency.

## Highway Autopilot

For a minimal highway autopilot stack, the vehicle only needs to know where it is with respect to
local features, such as lane markings, or where the vehicle was previously.
As such, such a stack would only require:
- Odometry
- TF manager

## Full stack

A full, redundant localization stack would require every component from the proposed localization
stack:
- Map manager
- TF manager
- Absolute Localizer
- Relative Localizer
- Odometry

Absolute localization can be used to initialize map management and relative localization. During
general runtime, observations odometry can be used in motion estimation. During
deprived scenarios, relative and/or odometry can be used to maintain minimal functionality
and restart absolute or relative localization when higher level sensing functionality is restored.

## SLaM

Simulataneous localization and mapping involves generating a map at runtime, and localizing against
this map. These algorithms typically involve an "inner" problem during which the localization
algorithm matches the current observation against recent history, and an "outer" problem, in which
the overall map is corrected for global consistency (i.e. bundle adjustment, pose graph
optimization).

In the context of the proposed localization stack, a component performing the SLaM task can be
considered to be an equivalent to an odometry component. If the outer problem produces global
estimates with respect to a known map frame, then it may also be considered a relative localizer
component.

## Mapping

Mapping generally involves aggregating observations and solving a global optimization problem to
ensure consistency between various optimizations, as stated in the SLaM case. In the case where
the mapping algorithms are kept separate from the registration/matching algorithms, a separate
mapping component can simply take in observations of the salient type and topic, and the transform
tree at a given time step.

The observations can then be transformed to produce an initial guess of the global map, upon which
refinement can be done in a non-real-time loop. The result may be stored somewhere.

It is generally not recommended to use the results of an asynchronous mapping procedure in a safety-
critical use case.

# References

- [1] [Vehicle Localization with low cost radar sensors, Ward and Folkesson](https://kth.diva-portal.org/smash/get/diva2:972553/FULLTEXT01.pdf)
- [2] [The Bingham Distribution of Quaternions and its Spherical Radon Transform in Texture Analysis, Kunze and Schaeben](https://www.researchgate.net/publication/226385995_The_Bingham_Distribution_of_Quaternions_and_Its_Spherical_Radon_Transform_in_Texture_Analysis)
- [3] [REP105 - Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
