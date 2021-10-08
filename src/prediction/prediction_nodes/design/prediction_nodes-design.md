Prediction nodes {#prediction_nodes-package-design}
===========

This is the design document for the `prediction_nodes` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This package contains one ROS node implemented in `autoware::prediction::PredictionNode`.

Start it for example with the provided launch file:

```console
$ ros2 launch prediction_nodes prediction_nodes.launch.py
```

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->


<!-- ## Assumptions / Known limits -->
<!-- Required -->

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
### Input parameters
- *time_step_ms*, default: 50 milliseconds
- *time_horizon_ms*, default: 3000 milliseconds

For further details, check @ref lonely_world_prediction-package-design and `autoware::prediction::Parameters`.

### Inputs

| input topic | input type | note |
|-----|---|---|
| **triggering** `/tracked_objects` | [`TrackedObjects`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/TrackedObjects.idl) |   |
| `/route`                          | [`Route`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/Route.idl)   |   |
| `/tracked_signals`                | [`TrafficSignalArray`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/TrafficSignalArray.idl)                                                                                                                                 |   |

The node additionally connects to the map server providing the `HADMapService`.

### Outputs

| output topic | output type |
|-----|---|
| `/prediction/predicted_objects` | [`PredictedObjects`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/PredictedObjects.idl)  |

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
Compared to the @ref prediction-architecture, this node currently misses the following:
-# use the map
-# transform inputs
-# map and motion based lonely-world prediction
-# scene interpretation
-# conflict resolution

# Related issues
<!-- Required -->
- #986 Add initial version of package
