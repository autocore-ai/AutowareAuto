Prediction architecture {#prediction-architecture}
=======================

@tableofcontents

The purpose of the prediction module is to provide future trajectories of dynamic objects in the scene that are relevant for planning.

# Scope

This document gives an overview of the prediction module for the cargo-delivery ODD developed in Autoware.Auto.

Assuming that
- the complexity of the ODD in terms of prediction needs is limited compared to inner-urban robotaxi applications,
- that large data sets to train machine-learning models for the ODD are not available

a rule-based approach is suggested as a good trade-off of implementation effort versus expected system performance.

# Requirements

1. Consistent inputs are used in every phase of the prediction; e.g.
    1. The input and output transformation should be based on the same information; i.e., the same localization and odometry information is used, even if an update to either becomes available after input transformation but before the output transformation is performed
1. Each object is predicted for the configurable time horizon.
1. The runtime of the entire module should remain below ~50 ms
1. Easy to accommodate different driving rules in countries/ODDs; e.g. left vs right-hand driving.

# Design

## Assumptions

As prediction improves, more information needs to be propagated within the subcomponents. It is therefore preferable to put them into separate C++ libraries called from one ROS2 node.

The prediction relies on the presence of an HD map and internally works in the map frame of reference.

The transformation of inputs and output could be done in a separate node, preferably running on the same ECU or even process as the core prediction. The transformation node needs a deterministic policy to choose the transformation to/from map coordinates in order to satisfy the *consistent inputs* requirement. But if  `tf2` is used, transformations may change in the background without the prediction node being aware of that. Hence it is easier to implement the transformation in a single C++ object that is owned by the prediction node.

The prediction module sits in a pipeline receiving input from the tracker and publishing output to the planning and control modules.

#### Map

The map needs to provide lanes with a center line and boundaries to be able to decide if an object is inside or outside a given area.

## Inputs / Outputs / API

### Inputs

- **triggering** tracked dynamic objects in odometry frame
- Lanelet2 HD map
- localization
- traffic-signal status
- previous ego plan

Once samples of the non-triggering inputs have arrived and the component reached a steady state, a
new iteration of prediction is started as tracked dynamic objects arrive, taking the samples for the
other inputs as they are available.

### Outputs

- predicted dynamic objects in odometry frame on topic `/prediction/predicted_objects`

## Inner-workings / Algorithms

The prediction is split conceptually into three phases

1. Scene interpretation
2. Lonely-world prediction
3. Conflict resolution

In phase 1., the available inputs are processed to extract higher-level features and to set the stage for the subsequent prediction.

In phase 2., the future behavior relating to an intent is generated taking into account

- the current (estimated) state of the object
- basic kinematic constraints
- the map

as if the object was the only dynamic object in the scene; i.e., in a lonely world.
In other words, the intents and states of other objects are **ignored** in this phase.

The output of phase 2. is a sequence of predicted kinematic states for each intent of each dynamic object spanning the time horizon.

In phase 3., conflicts or collisions between the ego vehicle and other objects generated in phase 2. are resolved according to the traffic rules and simple heuristics.

### Scene interpretation

The scene interpretation library can serve multiple purposes:

1. Association
  -# Given the lane association, infer one or more maneuvers, or intent, for each dynamic object. For example, for a vehicle waiting at a red traffic light, the intent may be to drive straight or to turn right. For a pedestrian near but not on a crossing, options are to cross or to follow a different path. This can be summarized as one or more sequences of lanelet IDs.
  -# Use map information to select dynamic objects for which the prediction needs to be modified or can be skipped; e.g. parked cars, pedestrians far from any lane.

The output of the scene interpretation includes a `PredictedDynamicObjects` message with information filled where applicable. Additional output includes  any other information such as the association of objects to lanes on the map that simplify the prediction in the next step.

### Lonely-world prediction

In a rule-based approach, the predicted trajectory is created from rules and heuristics. There are two main categories of prediction:

1. map-based prediction: object follows a lane in the map. Applied to vehicles, motorbikes, trucks etc. that are mapped to a lane.
2. motion-based prediction: object chooses a path regardless of lanes. Applied to pedestrians on a lane and to vehicles that are not associated with any lane.

As a simple but effective strategy in practice, the vehicles that are in lane are predicted to follow the center line subject to kinematic constraints. The lateral distance to the center line is predicted to diminish over time following an exponential decay with a heuristically chosen decay constant.

In a first iteration, only the geometrical center of an object is used for prediction. This leads to problems with objects that significantly differ from a point-like geometry such as articulated objects.

For motion-based prediction, the simplest approach is to extrapolate the current position in the direction of the current velocity. This leads to issues with pedestrians near the lane, especially when their orientation is not reliably estimated. These issues can be partially mitigated by the conflict resolution.

### Conflict resolution

The conflict resolution considers conflicts or collisions between the ego vehicle and other objects generated during the lonely-world prediction and resolves them according to the traffic rules and simple heuristics.

Conflicts between other agents are ignored; this should be good enough for the slow driving in the cargo delivery ODD and it speeds up the component.

This module takes the previous ego path as input. This reduces uncertainty as the lane association does not face any ambiguity and the planned path should be more accurate than a prediction with a simpler model.

The main use cases for the conflict resolution are

-# obeying the traffic rules at unprotected intersections so the ego vehicle yields and takes it right of way,
-# avoiding virtual conflicts with cars behind ego e.g. when waiting at a traffic signal.

For example, if ego wants to drive straight through an intersection and an oncoming vehicle wants to turn left such that the predicted paths collide, then the other vehicle's predicted path is modified to yield to ego.

### Traffic rules & local preferences.

In relying on the map for information regarding priorities of lanes, one can get into trouble if the map is out of date. But this allows to adapt to different driving rules (e.g. left-hand vs. right-hand driving) in various ODDs by simply exchanging an appropriate map.

Whenever there are differences in behavior prediction needed to handle local preferences, then they should be configurable via parameters. For example, if a vehicle is predicted to prefer the right lane on a multi-lane road, then "right" should be a parameter that can be changed at least in the initialization of the module.

## Known limits / Open questions
-# Interaction between other traffic agents is not considered
-# Interaction between ego and other traffic agents is simplified to conflict resolution and does not handle scenarios requiring co-operation such that ego and another agent have to leave their desired trajectory to jointly reach their target.
-# Not easy to predict articulated vehicles like a tractor trailer in tight curves. This requires a good perception to be able to combine the two objects into one and a good motion model. This is where rule-based approaches are stretched and it may be easier to solve with machine-learning approach.
-# Only a best estimate is output, no uncertainties are used on the input or output.
-# Static obstacles not considered
-# Occlusions not considered. The occluded area could be stored as an occupancy grid but that is currently unavailable in Autoware.Auto.
-# The map is taken as the static environment because there is currently no dynamic inference from perception.

# Error detection and handling

<!-- # Security considerations -->
TBD

# Future extensions / Unimplemented parts

## Machine learning

There is a limit to how many rules can be added to a rule-based prediction system before its complexity becomes unmanageable. An evolutionary step is to add and replace parts of the architecture with data-driven methods. For example, the scene interpretation could infer intentions based on [decision trees][GRIT] or neural networks. Another alternative is to use a neural network to generate one or more trajectories or to estimate parameters of a [Gaussian distribution describing the trajectory][Jens Schulz]. A thorough overview of deep-learning based approaches is presented in [this review article][deep review]. A more in-depth analysis of a few selected models is provided in [this report][Kao report].

A next level of sophistication is to omit the lonely-world prediction and to consider interactions directly, for example in a deep neural network like [GRIP++] or [TraPHic].

Even better performance in scenarios requiring to solve the interaction of other traffic agents with
ego requires to give up the modular approach and to integrate prediction with planning such that the
planner calls prediction to simulate different scenarios or planning and prediction are performed
simultaneously in an even tighter integration. An example of the latter may use reinforcement or
[imitation learning].

The final step of integration is to do end-to-end learning, creating control outputs from sensors inputs directly.

# References / External Links

- [Deep learning-based vehicle behavior prediction for autonomous driving applications: A review][deep review]
- [GRIP++: Enhanced Graph-based Interaction-aware Trajectory Prediction for Autonomous Driving][GRIP++]
- [GRIT: Verifiable Goal Recognition for Autonomous Driving using Decision Trees][GRIT]
- [An Algorithmic Perspective on Imitation Learning][imitation learning]
- [Interaction-aware probabilistic behavior prediction in urban environments][Jens Schulz]
- [Challenges and Tradeoffs in Trajectory Prediction for Autonomous Driving][Kao report]
- [Multipath: Multiple probabilistic anchor trajectory hypotheses for behavior prediction][Multipath]
- [Precog: Prediction conditioned on goals in visual multi-agent settings][Precog]
- [TraPHic: Predicting Trajectories of Road-Agents in Dense and Heterogeneous Traffic][TraPHic]

<!-- Reference links below are not rendered in output -->

[deep review]: https://arxiv.org/abs/1912.11676 "Deep learning-based vehicle behavior prediction for autonomous driving applications: A review"
[GRIP++]:  https://arxiv.org/abs/1907.07792v2 "GRIP++: Enhanced Graph-based Interaction-aware Trajectory Prediction for Autonomous Driving"
[GRIT]: https://arxiv.org/abs/2103.06113 "GRIT: Verifiable Goal Recognition for Autonomous Driving using Decision Trees"
[imitation learning]: https://arxiv.org/abs/1811.06711 "An Algorithmic Perspective on Imitation Learning"
[Jens Schulz]: https://arxiv.org/abs/1804.10467 "Interaction-aware probabilistic behavior prediction in urban environments"
[Kao report]: https://www2.eecs.berkeley.edu/Pubs/TechRpts/2020/EECS-2020-111.pdf "Challenges and Tradeoffs in Trajectory Prediction for Autonomous Driving"
[Multipath]: https://arxiv.org/abs/1910.05449 "Multipath: Multiple probabilistic anchor trajectory hypotheses for behavior prediction"
[Precog]: https://openaccess.thecvf.com/content_ICCV_2019/papers/Rhinehart_PRECOG_PREdiction_Conditioned_on_Goals_in_Visual_Multi-Agent_Settings_ICCV_2019_paper.pdf "Precog: Prediction conditioned on goals in visual multi-agent settings"
[TraPHic]:  https://gamma.umd.edu/researchdirections/autonomousdriving/traphic/ "TraPHic: Predicting Trajectories of Road-Agents in Dense and Heterogeneous Traffic"
