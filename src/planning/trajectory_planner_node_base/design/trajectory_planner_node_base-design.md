TrajectoryPlannerNodeBase {#trajectory-planner-node-base}
===========

This is the design document for the `trajectory_planner_node_base` package.


# Purpose / Use cases
A node with boilerplate to operate a trajectory planner to manage common input and output data flow across trajectory planners.

# Design

TrajectoryPlannerNodeBase is a generic trajectory planner node that returns trajectory that navigates from start state to goal state defined in request from the behavior planner.

Following functions must be implemented in the child class:
* `MapRequest create_map_request( route )` must be implemented. Return value should be the request message to map_provider. The implementer should create a request so that planner can receive sufficent map information for planning trajectory. For example, if parking planner might want to request parking spaces and lane objects in small region, whereas lane planner might want to query larger map with only lane objects.

* `Trajectory plan_trajectory( route, map )` must be implemented. Return value should be the trajectory that naviates from start state to goal state.

* `bool is_trajectory_valid( trajectory )` can be overriden to allow planner specific validation of trajectory.

## Assumptions / Known limits
<!-- Required -->
Currently trajectory message has bounded capacity. If trajectory requires more points than the capacity, it must be cropped so that the size fits within the capacity. It is the behavior planner's job to redo the request to each planner if previously planned trajectory is incomplete.

## Inputs / Outputs / API
Input:
* Trajectory request with start and goal state
* HAD Map via service call

Output:
* Calculated Trajectory
* ReturnCode of the planner (SUCCESS/FAIL)

## Inner-workings / Algorithms


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
<!-- Optional -->
* Final validation of trajectory before publishing must be implemented. This can be done as more planner is implemented so that we have better view about what validation is needed and can be made common.

# Related issues
<!-- Required -->
