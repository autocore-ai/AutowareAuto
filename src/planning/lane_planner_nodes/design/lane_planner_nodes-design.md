Lane planner nodes {#lane-planner-nodes}
===========

This is the design document for the `lane_planner_nodes` package.


# Purpose / Use cases
The LanePlannerNode is a ROS wrapper class for lane_planner.

# Design
This inherits TrajectoryPlannerNodeBase class which defines common interface among different planners.

Implemented virtual functions:
* create_map_request: currently it requests for full map since have no way of knowing the size of map that includes all given map_primitive in the route.
* plan_trajectory: passes the input to LanePlanner class to plan trajectory.

## Assumptions / Known limits

## Inputs / Outputs / API
See TrajectoryPlannerNodeBase for inputs and outputs

## Inner-workings / Algorithms
<!-- If applicable -->


## Error detection and handling
<!-- Required -->


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
* reimplement createMapRequest function with planning_window size.

# Related issues
<!-- Required -->
