State Estimation Node {#state-estimation-nodes-design}
=====================

# Purpose / Use cases

We need a node for state estimation within Autoware Auto. This node uses `kalman_filter` package as the backbone.

# Assumptions
For now we assume that tracking happens in 2D and that the Constant Acceleration motion model is used.

Note that the outgoing messages will be timestamped in the same time reference frame as the incoming messages, and we assume that all clocks that timestamp the incoming messages are properly synchronized. Furthermore the Kalman filter predictions will happen on a steady time grid initialized by the first received state and an expected interval between predictions.

# Input / Output

The inputs are measurements that update the prediction of the underlying filter estimate. Currently, the node supports the following inputs:
- `geometry_msgs/msg/PoseWithCovariance` - provides the position and orientation
- `autoware_auto_msgs::msg::RelativePositionWithCovarianceStamped` - provides the position
- `geometry_msgs/msg/TwistWithCovariance` - provides a speed measurement
- `nav_msgs/msg/Odometry` - provides both position and speed measurements

@warning The `frame_id`s of all the incoming messages must match the `frame_id` provided as a parameter to this node. It is the task of the publishing nodes to provide the data in the correct frame.

@warning It is assumed that all messages measure the state of the same frame, i.e., a `child_frame_id` for all incoming messages is the same. For messages that don't have a `child_frame_id` field, the user is responsible to enforce these messages to also measure the state of the same frame as other messages.

There can be multiple topics for this node and these must be configured through the parameters.

The state estimator provides the following output:
- filtered position, orientation and linear and angular velocity as `nav_msgs/msg/Odometry` on the topic `filtered_state` that can be remapped by the user.

@note We will only focus on time-stamped messages here.

# Implementation details
The core functionality of this node resides in the `KalmanFilterWrapper` class. To initialize this class we need the following:
- a square (usually diagonal) matrix with the variances for our state variables
- a rectangular matrix for the process noise. For example, for 1D case it can be [0, 0, 1], meaning there is only noise on the acceleration.
- expected time between updates of the filter. This is used to initialize a grid of time at which we expect the node to publish predictions if a timer-based approach is used.
- mahalanobis distance to reject outlier measurements
- motion model that we use internally in our EKF.

This class provides a high-level interface to use potentially different Kalman Filters implementations under the hood, configuring them through the template parameters of this class. It supports all the classical operations of the Kalman Filter such as prediction, update (in this case from ROS messages) as well as getting the state and its covariance as a ROS message.

@note The filter will not predict the state before it has seen a stateful observation. After that it works as intended.

## History to deal with out-of-order measurements
All "events" (e.g. reset, measurement update, prediction) are stored in a history of events. It is organized as a queue by time. Whenever a new event arrives it is placed into the queue at the place indicated by its timestamp and the events that are now later in the queue get "replayed" on top of the current event, thus updating the last estimated state in the queue.

### Example
Let's say we have a history of maximum 5 events. The events can be Reset (`R`), Predict (`P`), and Update (`U`). The events are stored in the history sorted by their timestamp and there is a state vector assigned to each event that represents the state at that timestamp (`S0` - `S8`).

Let's assume the history is currently in the following configuration:

```
timestamp:     0    2    4    6    8
events:     ---R----P----U----P----P--->
state:         S0   S2   S4   S6   S8
```
Then, there is a new Update coming at time 5 like so:

```
timestamp:     0    2    4    6    8
events:     ---R----P----U----P----P--->
                           ^
                           U
```
The history can only hold 5 events so the oldest one will have to be dropped and the new inserted. All the following events will update their accompanying state taking into account the new observation at time 5. The updated states are denoted with S6' and S8' in the diagram below:
```
timestamp:     2    4  5  6    8
events:     ---P----U--U--P----P--->
state:         S2   S4 S5 S6'  S8'
```

@note The history-based update means the output of the filter _is not continuous_, strictly speaking. However, the discontinuities are likely to be negligibly small. If this proves to not be the case, we would need to opt for a more complex approach to deal with the out-of-order measurements.

