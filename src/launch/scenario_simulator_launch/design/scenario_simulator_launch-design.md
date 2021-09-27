scenario_simulator_launch {#scenario_simulator_launch-package-design}
===========

# Purpose / Use cases

This package provides a launch file for running Autoware.Auto with scenario simulator. Scenario simulator is a tool that enables running user-defined scenarios to test the autonomous-driving stack behavior in certain situations. It can be used manually by developers and testers or it can be integrated with CI.

# Design

A scenario describes the situation to be created by defining:
* environment;
* ego vehicle: initial state and desired behavior;
* other agents (NPCs) in the environment, their initial state, and their behavior.

Based on the description provided by the scenario the situation is created in a simulated environment. Ego vehicle is controlled by the autonomous-driving stack using the launch provided by this package. NPCs are controlled by the simulator. A course of the scenario is constantly supervised. The scenario finishes with success (for example when the ego vehicle reaches the goal) or with failure (for example when ego hits a pedestrian or when scenario times out) and the result is reported.

This package launches only the required stacks: mapping, perception, and planning.

Note that this package does not provide the scenario simulator itself but only the launch for the simulator.

# References / External links
- https://en.wikipedia.org/wiki/Scenario_testing
- https://github.com/tier4/scenario_simulator_v2

# Related issues
- Issue [#1215](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1215): Create scenario testing framework
