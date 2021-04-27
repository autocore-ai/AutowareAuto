State and Variables {#state-and-variables-design}
=============

# Purpose / Use cases

## Motivation
Any form of state estimation requires a form of representing a state. A common choice is to have state represented in the form of an array of some sort. For example, a state consisting of variables `X`, `Y`, `Z` would be represented by the state vector of size 3. This is a useful view but lacks expressiveness and causes issues in code when accessing variables. Also, this representation is overly generic as there is no way of telling apart a state vector representing `X`, `Y`, `Z`, and for example `roll`, `pitch`, `yaw` - both would be implemented as a vector of size 3.

## Proposed design
In this package the state is represented through concretely typed variables. Basically, anything inheriting from a simple tag struct `autoware::common::state_vector::Variable` is a variable. There is an accompanying trait to show this. The variable types can then be provided as template parameters into the state type.

This allows to then use the `autoware::common::state_vector::CommonState` type to instantiate this concretely typed state. Now the states representing different variables will actually have a different type. Under the hood, an instance of such a state is still going to own a vector of appropriate size to store its values. These values can be then queried by index or by variable. The current implementation additionally provides typically used variables in `common_variables.hpp` and typically used states in `common_states.hpp` files.

## Example usage
```cpp
struct X : Variable{};
struct X_VELOCITY : Variable{};
struct Y : Variable{};
struct Y_VELOCITY : Variable{};
struct YAW : AngleVariable{};
using State = FloatState<X, X_VELOCITY, Y, Y_VELOCITY, YAW>;

State state{};
state.at<X_VELOCITY>() = 1.0F;
state.at<Y_VELOCITY>() = 1.0F;
```

# References

[`#865`](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/865) - Redesign kalman filter class hierarchy
