MPC Controller Design {#mpc-controller-design}
=============================================

# Purpose

Model predictive control (MPC) involves solving an optimization problem on demand and using
the initial parts of the generated solution to plan or control the system for the next
few time steps. This technique is good for determining control inputs as it is aware of
the dynamics of the modeled system, and thus can choose a control action that is optimal
for not just the immediate state, but to also set up future states for success.

This package seeks to implement a MPC controller for a single track vehicle platform.

# Design

## Vehicle Dynamics Model

A basic kinematic bicycle model is used.

## Behavior

Receding horizon model predictive control is implemented.

When receiving a new trajectory, the next command is computed via a cold-started optimization
problem. This means that the initial guess is the reference trajectory with no control inputs.

As the vehicle advances along a reference trajectory, previous solutions are rolled forward
(warm starting), and reference points are back-filled from the refrence trajectory.

As the reference trajectory nears the end, reference weights are appropriately zeroed out to
acheive the receding horizon behavior.

## API

This controller uses the ControllerBase interface. As such, if the current state is "past" the
current trajectory in some sense, a stopping deceleration command will be provided.

In all other cases, the library will solve an optimization problem and use the first element of
the plan as the control command.

## Implementaiton

static_asserts are used as a coarse smell test for whether generated code has changed from the
facade. They are generally placed around the code that might be affected.

# Future improvements

- Trajectory interpolation
- Further input validation

## Dynamic Vehicle Model

A dynamic vehicle model should be used to more accurately represent the vehicle motion at
higher velocities.

The model has been adapted from [1].

The following adaptations have been made:
- Longitudinal velocity is not assumed to be constant
- Heading angle with respect to a (fixed) intertial frame is included in the state vector
- Acceleration is included as a state variable, and assumed to act directly on longitudinal
acceleration
- Jerk and wheel angle turn rate were used as the control control inputs

These adaptations were made for the following reasons:
- To support acceleration and deceleration trajectories
- To support arbitrary trajectory orientations
- To improve smoothness of the resulting control plans



# References

[1] [Planning Algorithms, LaValle](http://planning.cs.uiuc.edu/node695.html)
