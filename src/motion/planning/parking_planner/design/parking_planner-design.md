Parking Planner Design {#parking-planner-design}
================================================

# Purpose
The purpose of the parking planner is to plan complex maneuvers for moving a vehicle into and out of parking spots, avoiding obstacles along the way.

# Design

## Vehicle Dynamics Model

A basic kinematic bicycle model is used as presented in [1], extended with a state for the front wheel steering angle.

## Behavior

The parking planner provides a method for planning a dynamically feasible trajectory from a given starting vehicle state to a given target vehicle state. 
The trajectory avoids obstacles and is provided as a list of both vehicle commands as well as vehicle dynamic states. 
The vehicle commands lead to the corresponding states when applied in a zero-order-hold fashion.

## API

The parking planner provides a function `plan`, taking initial and goal states as well as obstacle polyhedra as parameters. 
The function then returns an object containing the resulting trajectory as well as some metainformation about the solution process.

## Implementation

The planner follows the method presented in [2] and involves multiple stages of computation:

1. An A\* global planner is run to obtain a vector of states that is not dynamically feasible yet, but provides a rough discretized collision-free path from start to finish if the A\* was successful. 

2. The A\* path is resized to match the horizon set in the NLP solver and augmented with zero inputs into a full state-and-input trajectory initial guess

3. The NLP solver is called to obtain a smooth, dynamically feasible trajectory that avoids obstacles and takes the vehicle from the initial state to the target state. 

4. The resulting trajectory is checked again for constraint and dynamics satisfaction as well as lack of collisions.

# References

[1] [Kinematic bicycle model paper](https://www.researchgate.net/profile/Philip_Polack/publication/318810853_The_kinematic_bicycle_model_A_consistent_model_for_planning_feasible_trajectories_for_autonomous_vehicles/links/5addcbc2a6fdcc29358b9c01/The-kinematic-bicycle-model-A-consistent-model-for-planning-feasible-trajectories-for-autonomous-vehicles.pdf)

[2] [Zhang, Liniger, Borelli](https://arxiv.org/pdf/1711.03449.pdf)
