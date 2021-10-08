Lonely-world prediction {#lonely_world_prediction-package-design}
===========

This is the design document for the `lonely_world_prediction` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

As part of the @ref prediction-architecture, the purpose of the lonely-world prediction is to
predict dynamic objects as if they did not interact with each other.

This package provides C++ functions to be called from ROS nodes.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

In the first iteration, it is only possible to predict objects to be stationary. Include the main header with

```cpp
#include "lonely_world_prediction/lonely_world_prediction.hpp
```

and call

```cpp
void predict_stationary(
  autoware_auto_msgs::msg::PredictedObjects & predicted_objects,
  const autoware::prediction::Parameters & parameters);
```

The input `predicted_objects` are modified in place and contain the predicted paths on return. Each
path consists of replicas of the initial pose for each time step. Any velocity or acceleration of
the initial state is preserved in the output but ignored for the purpose of prediction; i.e., the
prediction may be unphysical.

The
[`PredictedObjects`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/PredictedObjects.idl)
can be initialized from
[`TrackedObjects`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/TrackedObjects.idl)
using

```cpp
// #include "lonely_world_prediction/init_from_tracked.hpp"
autoware_auto_msgs::msg::PredictedObjects
from_tracked(const autoware_auto_msgs::msg::TrackedObjects &);
```

The [Parameters](@ref autoware::prediction::Parameters) define the
-# *time step*: time difference between two consecutive predicted poses
-# *time horizon*: the time difference from now into the future until which to predict objects

If `horizon MOD step > 0`, then an extra pose is added to the predicted path such that the
horizon is always covered.

## Assumptions / Known limitations
<!-- Required -->
-# The uncertainty due to an unknown sign of the orientation of an object is not transmitted
-# The predicted path only has one pose, even if the object has multiple shapes. This matches the tracker's known limit

<!-- ## Inner-workings / Algorithms -->
<!-- If applicable -->


<!-- ## Error detection and handling -->
<!-- Required -->


<!-- # Security considerations -->
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


<!-- # References / External links -->
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->
-# motion-based prediction
-# map-based prediction

# Related issues
<!-- Required -->

- #986 Add initial version of package
