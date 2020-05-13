Controller Design {#controller-design}
=================

A controller is an integral part of the autonomous driving stack. The controller operates at
high rates and ensures that the vehicle appropriate follows a reference trajectory, minimizing
error with respect to the pre-specified path or trajectory.

This document seeks to appropriately define the use cases and requirements for the controller
component of the autonomous driving stack. From this, a proper design and architecture for a
controller may be derived.

# Use Cases

This section seeks to define use cases that the controller software component must satisfy. This
section is broken into high-level and atomic use cases. The high level use cases are those the
entire autonomous vehicle (AV) stack must be able to satisfy. These high level use cases are then
used to derive atomic use cases that the controller in isolation must be able to satisfy.

## High Level Use Cases

The following use cases are presented at different levels of Autonomy, beginning at 2 (longitudinal
and lateral control) up to 5 (full autonomy).

A collection (non-exhaustive) of illustrative use cases are present below:
- The vehicle must be able to navigate safely at low speeds (<= 20 kph) (2-5)
- The vehicle must be able to navigate safely at high speeds (> 20 kph) (2-5)
- The vehicle must be able to maneuver into narrow spaces (i.e. pull-in, parallel parking maneuvers)
(3-5)
- The above use cases must be achievable on varying vehicle platforms

The ability to navigate safely then consists (non-exhaustively) of the following use cases:
- The vehicle must be able to stay within a specified drivable space (e.g. a lane) (2-5)
- The vehicle must be able to stop for an object (real or virtual, e.g. a stop line) within its
driving envelope (2-5)
- The vehicle must be able to avoid obstacles within its drivable space (3-5)

## Atomic Use Cases

Each of the high level use cases are then broken down appropriately.

Each stated use cases relating to safe navigation can be achieved, predicated on the
assumption that an appropriate trajectory can be planned such that all of the above cases are
satisfied.

The controller's responsibility for safe maneuvering then boils down to the following use case:
- Given a set of inputs, the controller must be able to achieve the specified sequence of dynamic
states with a bounded level of error

More specifically, given that a reference trajectory is a sequence of time-indexed dynamic states
(e.g. poses, velocities, accelerations), the controller must be able to generate a series of
actuation commands such that at each time step, the vehicle's actual dynamic state is within some
error bound with respect to the reference dynamic state.

Note that it is not specified what error metric is used, nor what error bound is used (at which time
step). The specific requirements for these bounds and metrics are left to the final system
integrator.

The ability to maneuver into narrow spaces then consists (non-exhaustively) of the following use
cases:
- The vehicle must be able to semi-holonomically achieve an implementation-defined subset of all
configuration states with a bounded level of error

# Requirements

Each of the atomic use cases can then commonly be broken down to a set of requirements.

A general assumption is made that the controller is not intended to achieve maneuvers near the limit
of vehicle performance. This assumption is made to simplify interfaces and make it easier for the
controllers to support multiple vehicle platforms.

An additional assumption is that the vehicle platforms are constrained to operate on a plane. That
is, for the purposes of planning and control, the vehicle is constrained to the SE(2) space.

## Bounded dynamic state error

The following assumptions are made for the general case of vehicle trajectory following:
- The dynamic state is assumed to have at least the following properties:
  - Position (at least x, y)
  - Velocity (at least longitudinal and lateral)

### Low speed

The following assumptions are made for the case of low-speed trajectory following:
- Tires do not slip

### High speed

The following assumptions are made for the case of high-speed trajectory following:
- Tires may slip

Based on the above assumption, the vehicle dynamics must be modeled. At a bare minimum this
implies that the tires must be modeled. Additional elements of the vehicle may or may not be
modeled, such as:
- Suspension
- Power train
- Vehicle mass

In order for a controller to successfully control a vehicle under the assumptions of a dynamical
vehicle model, the following information must be available in real time:
- Longitudinal velocity
- Lateral velocity
- Actuator pose (e.g. wheel angle)
- Yaw rate

Additional real time information may be used, such as:
- Longitudinal acceleration
- Lateral acceleration
- Actuation rates (e.g. wheel turn rate)

Finally, some additional information may be used at real time, though they may be assumed to be
(runtime) static:
- Vehicle mass
- Vehicle center of gravity
- Tire properties (i.e. steering gradient, etc.)
- Rotational/moment of inertia (with respect to the z axis)

## Achieving configuration states

The following assumptions are made for the case of achieving configuration states:
- The vehicle may move in both the forward and reverse directions
- The vehicle may be N-link
- The configuration state contains at least position (x, y), and heading

The most extreme configuration from any (static) state is one that is directly lateral from the
current pose (i.e. parallel parking with very little margin). This setting will be used as
inspiration for the following derived requirements.

Note that the equivalent setting of an N-point turn may also be used as inspiration.

To achieve a semi-arbitrary configuration state, the following requirements are needed:
- The rotation of the vehicle must be able to actuate to extreme positions with or without
longitudinal velocity
- The vehicle must be able to satisfy both positive and negative longitudinal
velocities/displacements (e.g. vehicle must be able to switch between the drive and reverse gear
settings)
- The vehicle must be able to produce some small, discrete unit of longitudinal displacement,
and accurately detect the satisfaction of this displacement with a bounded error

## Supporting varying vehicle platforms

The following assumptions are made for the case of supporting varying vehicle platforms:
- The actuation that may be controlled are all on a single rigid-body platform (i.e. for a N-link
vehicle, all wheels for which steering and throttle may be controlled all reside on a single rigid
link)
- A single-track approximation is appropriate for the vehicle platform (i.e. a form of
Ackerman is used for the front and/or rear wheels)
- Throttle or acceleration is assumed to be a bulk parameter that can be applied to the vehicle
platform as a whole (i.e. as opposed to being applied as torque on a per-wheel basis)

These assumptions are generally made to simplify modeling assumptions and thus facilitate the
decoupling of the motion planner, controller, and vehicle interface via reasonably simple
interfaces.

If any of these assumptions must be broken, then one of the following is likely true:
1. The vehicle is a platform with highly non-standard dynamics
2. The user expects the vehicle to operate near or at the limits of it's dynamics

In all such cases, it may be more appropriate for a custom planning architecture to be used specific
for the specialized use case (e.g. involving tighter coupling of the planner, controller,
vehicle interface, and vehicle platform).

# Mechanisms

In order to satisfy requirements, the following inputs, outputs are needed by the controller.
In addition, the controller must also be able to satisfy a certain set of behaviors.

## Behaviors

The fundamental behavior the controller must follow is:
- Generate a control sequence that produces a sequence of (dynamic) states that is bounded
in error with respect to the reference trajectory

This requirement can be achieved by choosing an appropriate controller algorithm that satisfies
error rejection, and either proving theoretically or sufficiently testing to establish error bounds.

Some additional behavioral requirements that may be necessary to satisfy this fundamental behavior
may include:
- Acquiring and sorting transforms such that the ego state may be transformed into a frame
compatible with the provided trajectory

Note that these requirements are optional and subject to the final discretion of the system
integrator.

Additional behaviors the controller may support include:
- System identification of center of gravity, other assumed runtime constants; estimation of error
- Diagnostics for when dynamic state or system model error levels exceed some threshold (i.e.
protection level on these errors)

## Inputs

The following inputs are required:
- Ego state information:
  - Position (x, y, heading) transformable to the trajectory frame (all use cases)
    - Transforms to satisfy above input
  - Longitudinal velocity (safe maneuvering use case)
  - Lateral velocity (high speed safe maneuvering)
  - Yaw rate (high speed maneuvering)
  - Wheel angles (all use cases)
- A reference trajectory with at least:
  - Position
  - Longitudinal velocity
  - Wheel angles
  - Yaw rate (high speed maneuvering)

The following inputs are optional:
- Longitudinal acceleration (high speed maneuvering)
- Lateral acceleration (high speed maneuvering)
- Wheel angle rate (high speed maneuvering)

The following inputs are needed, but may be considered runtime static (e.g. a configuration
parameter):
- Vehicle mass/inertia
- Vehicle center of gravity
- Vehicle tire properties
- Yaw/moment of inertia

## Outputs

The following outputs are required:
- Longitudinal control command
- Lateral/steering command

The following optional outputs may be provided to support optional behavior:
- Trajectory-following error statistics
- System identification error statistics or parameters

# Design

The following design considerations are then proposed:

## Inputs

The inputs to the controller shall be three types:
1. The vehicle state (At a high rate, the triggering topic)
2. The trajectory (At a lower rate, secondary topic)
3. Transform messages

**Rationale**
1. Inputs (1) and (2) are strictly necessary for the core function of the controller
2. Input (3) is needed to ensure that input (1) can be put into a compatible state with
input (2)

### Vehicle State

The vehicle state type shall contain **at least** the following information:
1. x position (of the **center of gravity** with respect to some coordinate frame)
2. y position (of the **center of gravity** with respect to some coordinate frame)
3. Orientation (as an imaginary number)
4. An identifier for the reference coordinate frame (e.g. string `frame_id`)
5. Longitudinal velocity (of the **center of gravity** with respect to some **assumed static**
coordinate frame)
6. Lateral velocity (of the **center of gravity** with respect to some **assumed static**
coordinate frame)
7. Wheel angles (front and rear, assuming single-track model)
8. A time stamp, which is the time point corresponding to the time at which the original sensor data
was generated
9. Yaw rate

All parameters are in standard ISO units, where applicable (meters, seconds, kilograms, radians).

**Rationale**
1. The vehicle pose (x, y position, orientation) is with respect to the center of gravity because
these coordinates can easily be translated for use with a kinematic model (see [1]), while the
opposite is not true for the case of a kinematic model to a dynamical model. A dynamical vehicle
model is necessary for the high speed maneuvering use case.
2. Orientation (3) is represented as an imaginary number to simplify computation of sines and
cosines, and reduce ambiguity between states in SO(2).
3. Pose is given with respect to a coordinate frame so that it may be unambiguously made compatible
with the dynamic states given by the trajectory. This is strictly necessary for basic functionality
4. Longitudinal velocity (5) is minimally needed to more directly close the control loop, given that
the longitudinal control is that of acceleration (the next derivative)
5. Lateral velocity (6) is needed to estimate the dynamical effects of the vehicle and/or to fully
specify the vehicle's state during dynamic maneuvers. This is needed for the high speed maneuvering
use case, and for the transition to and from the dynamic vehicle model.
6. Wheel angles (7) are specified instead of path curvature because specification of wheel/tire
dynamics are needed for the high speed maneuvering case. While curvature can be computed via wheel
angles, the same is not strictly true in the reverse direction.
7. Where equivalent representations are available in both the dynamic and kinematic control case
(such as reference coordinate, curvature representation, etc.), preference is given to the dynamic
case. This is because the dynamic case occurs at higher velocities, where lower latencies, and thus
less intermediate computation may be preferred.
8. Wheel angles (7) are used in the state representation because an implementation of vehicle
state estimation may use vehicle odometry. Wheel angles are further used instead of curvature
because the steering dynamics of the vehicle may have more than one degree of freedom, which cannot
be unambiguously represented by curvature
9. Velocities (5), (6), are assumed to be with respect to a static coordinate frame because most
relevant frames (e.g. `/world`, `/map`) can be considered static for the purposes of planning and
control. This assumption further simplifies processing and allows us to use standard transform
messages
10. A time stamp (8) is needed to ensure that secondary values can properly be aligned in time (e.g.
via interpolation)
11. The time stamp is the same as the originating sensor data so that data can be properly aligned
when the results arrive at different latencies
12. Only the front and rear wheel angles are expected in order to provide some level of
extensibility, e.g. to rear-wheel steering platforms, while keeping with the assumption of only
a rigid-body platform being actuated. Independent steering of all wheels on a platform is outside
the scope of the vehicle controller as that would require complicated dynamics likely approaching
the limits of the real vehicle platform

### Trajectory

The trajectory type shall contain **at least** the following information:
1. An identifier for the reference coordinate frame (e.g. string `frame_id`)
2. A bounded sequence of trajectory points, with **at least** the following information:
      1. All information represented in vehicle state (except for redundant frame identifier and
      time stamp), with the same semantics
      2. A time duration representing the target time for the given dynamic state
3. A time stamp

**Rationale**
1. A coordinate frame identifier (1) is needed so that the observed vehicle state can be put into a
consistent coordinate frame with the trajectory
2. A time stamp (3) is needed so that each point in the trajectory is rooted in time. This can aid
path following in between receipt of new trajectories.
3. Trajectory points (2) are formed from vehicle states (2a) to ensure a compatible representation
for the purposes of measuring, bounding, and rejecting error. Additionally, this reduces the
representational overhead. Similar reasoning can be applied for requiring the same semantics
4. A time duration (2b) is needed for each trajectory point so that each point of the trajectory can
be unambiguously rooted in both space and time.
5. A bounded representation of the trajectory is needed to support the static memory requirement
that is a mechanism to satisfy the safety-critical use case.
6. The time stamp is the same as the originating sensor data so that data can be properly aligned
when the results arrive at different latencies. In addition, this information can be used to "root"
a trajectory if the reference frame is non-static (e.g. level 3 highway driving case)

### Transforms

A transform type shall contain at least the following information:
1. Parent coordinate frame
2. Child coordinate frame
3. Time stamp
4. Translation (meters)
5. Rotation

The `geometry_msgs::TransformStamped` type or any higher-level messages composed of this type
satisfies these requirements.

**Rationale**
1. A rigid-body transformation allows us to convert vehicle state estimates between coordinate
frames
2. A time stamp allows coordinates in moving frames to be transformed

## Outputs

The outputs from the controller shall be the following type:
1. Control command (reference values)

Additional types may be output in order to satisfy optional behaviors:
1. Error statistics
2. System identification parameters, which may contain parameters which are assumed to be runtime
static (such as mass), and the error of these relevant parameters

### Control commands

The control command type shall contain at least the following information:
1. Wheel angles (front and rear, assuming single track model)
2. Acceleration

These parameters should have the semantics of being considered a **reference value**.

**Rationale**
1. Control commands are considered reference values because the underlying vehicle platform may
contain controllers on embedded ECUs. These values cannot be considered feed-forward values without
strong assumptions about the vehicle interface and vehicle platform itself, which may not be
satisfiable in the general case.
2. Wheel angles assume a single track model with Ackerman-like steering to simplify modeling
assumptions.
3. Front and rear steering angles are provided to handle front, rear, and independently steering
platforms. Additional degrees of steering freedom are not considered because we assume actuation
only occurs on a single rigid-body platform. The inclusion of additional degrees of freedom would
introduce significant tire slip, and effective use of such degrees of freedom would likely require
the vehicle to be operating near its maneuvering capacity.
4. Desired bulk acceleration is provided instead of individual wheel angles to simplify the modeling
assumption and interfaces. If individual wheel torque is needed, this may be delegated to internal
controllers in the vehicle interface level, or external ECUs. Introducing individual wheel torques
would greatly complicate the modeling required at both the controller and motion planning layers.

### Error statistics

The error statistic type should contain the following information:
1. Error for each of the relevant vehicle state fields

**Rationale**
1. Error statistics can provide a measure of controller performance. This can be used to compare
controller performance between implementations.
2. Error statistics can be used during runtime to determine if the vehicle is operating within
expected performance bounds. If the errors exceed some bounds, a warning or takeover alert may
be raised.
3. Error statistics for all elements of the vehicle state are relevant as the core function of
the controller is to reject error between (all elements of) the reference state and the ego
state. All error information is useable. For example, errors in higher positional derivatives can
be leading error indicators.

### System identification statistics

The system identification statistics type should contain the following information:
1. Estimates for (assumed) runtime static parameters, e.g.:
    1. Vehicle mass
    2. Vehicle center of gravity
    3. Tire properties (i.e. steering gradient, etc.)
2. An error measure, based on deviation of estimated parameters from used parameters


**Rationale**
1. The controller error statistics can be used to estimate degrees of freedom in the dynamic
model specification, where relevant
2. This information may be used to close the loop for highly dynamic planning

# References

[1] Reacting to Multi-Obstacle Emergency Scenarios Using Linear Time Varying Model Predictive
Control. Jain et al.
