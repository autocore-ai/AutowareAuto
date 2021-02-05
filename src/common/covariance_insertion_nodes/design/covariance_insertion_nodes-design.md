Covariance Insertion Node {#covariance-insertion-nodes-design}
=========================

# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

This node is designed to subscribe to a provided topic (`messages` by default), add covariance to an existing field republish the message to a topic `<input_topic>_with_overriden_covariance`, by default being `messages_with_overriden_covariance`. If the input message does not have covariance field an equivalent message with covariance would be filled. For example:
- If the `input_msg_type` is `Odometry` it will be republished with covariance overridden. The same holds for types that already contain covariance, like `PoseWithCovariance` or `TwistWithCovariance` and their stamped versions.
- If the `input_msg_type` does not have covariance in it, like `Pose` or `PoseStamped` it will be republished as `PoseWithCovariance` or `PoseWithCovarianceStamped` message respectively. The same should hold for `Twist` and its derivatives.

A possible use-case for such a node is to fill in the covariance values for Odometry message when this message is used in the StateEstimatorNode and the original provider of this message does not fill the covariances. The StateEstimationNode requires the covariances to be valid to function so we can use this node to fill them with a value that makes sense to us as an explicit engineering factor.


# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

The node is designed to allow it to be extended to any message type. It requires a single topic, its input type and an entry that specifies all entries of the covariance.

For example, for `Odometry` messages example above this can be:

```json
input_msg_type: "Odometry"
override_covariance: 
    pose: [42.0, 42.0, ...]  // 36 values for a full covariance matrix
    twist: [42.0, 42.0, ...]  // 36 values for a full covariance matrix
```

## Assumptions / Known limits
<!-- Required -->
This only works to add covariances to messages and works with a limited set of messages. It is possible to make this more general, but this requires a fair amount of engineering so for now it is not implemented in the most general way. That being said, it should be possible to extend the code for any messages that can hold covariances.

## Inner-workings / Algorithms
<!-- If applicable -->
The node will first guess the output type then will create a publisher and subscriber for proper input type and output types. From this point on, every incoming message will be updated with the covariance and published as an output type.

## Error detection and handling
<!-- Required -->
Most errors can step from a missing (not implemented) type trait or using the node with the type that is not yet there. These errors should crash the node to notify the user that the node cannot work with the specified types or parameters.

# Future extensions / Unimplemented parts
<!-- Optional -->
The node can be extended to work with more types. See `covariance_insertion::convert.hpp`, `covariance_insertion::add_covariance.hpp`.

# Related issues
- #505 - Integrating a state estimator with NDT
