State Estimation Node {#state-estimation-nodes-design}
=====================

# Purpose / Use cases

We need a node for state estimation within Autoware Auto. This node uses `kalman_filter` package as the backbone and provides interface similar to the [robot_localization][1] package, albeit following a different design.
 
# Assumptions
For now we assume that tracking happens in 2D and that the Constant Acceleration motion model is used. The code is designed in a way to allow for configuring these at a later point in time.

Note that the outgoing messages will be timestamped in the same time reference frame as the incoming messages, and we assume that all clocks that timestamp the incoming messages are properly synchronized.

# Input / Output

The inputs are measurements that update the prediction of the underlying EKF estimate. Currently, the node supports the following inputs:
- `geometry_msgs/msg/PoseWithCovariance` - updates the 2d position
- `geometry_msgs/msg/TwistWithCovariance` - updates the 2d speed
- `nav_msgs/msg/Odometry` - updates both position and speed

The state estimator provides the following output: 
- filtered position, orientation and linear and angular velocity as `nav_msgs/msg/Odometry` on the topic `filtered_state` that can be remapped by the user.

**Note:** we will only focus on time-stamped messages here.  

# Implementation details
The core functionality of this node resides in the `KalmanFilterWrapper` class. To initialize this class we need the following: 
- a square (usually diagonal) matrix with the variances for our state variables
- a rectangular matrix for the process noise. For example, for 1D case it can be [0, 0, 1], meaning there is only noise on the acceleration.
- expected time between updates of the filter. This is used to generate the proper GQ factor.
- mahalanobis distance to reject outlier measurements
- motion model that we use internally in our EKF as well as for generating the GQ factor. In our case, the GQ factor is generated as the Jacobian of our motion model multiplied by the process noise matrix, which follows Section 6.3.3, page 274 of the [Estimation with Applications to Tracking and Navigation][2] book.

This class provides a high-level interface to use potentially different Kalman Filters implementations under the hood, configuring them through the template parameters of this class. It supports all the classical operations of the Kalman Filter such as prediction, update (in this case from ROS messages) as well as getting the state and its covariance as a ROS message. 

**Note:** The filter will not predict the state before it has seen a stateful observation. After that it works as intended.

## Math recap
Just as a short recap, following this [discussion][3].

For a state of position, velocity and acceleration we have the following representation of the transition funtion and state vector (in a 1D case):
\f[
    F = \left[\begin{matrix}1 & dt & \frac{dt^{2}}{2}\\0 & 1 & dt\\0 & 0 & 1\end{matrix}\right],\hspace{5mm}x = \left[\begin{matrix}x\\v\\a\end{matrix}\right]
\f]
The temporal update to the state covariance looks like this (I am assuming that the \f$Q\f$ matrix does not change with time for simplicity of notation):
\f[
    P_t = F P_{t-1} F^\top + G Q G^\top
\f]

The parts that might get confusing:
- The [Bar-Shalom book](https://www.amazon.com/Estimation-Applications-Tracking-Navigation-Bar-Shalom/dp/047141655X) in its Square Root filtering chapter on page 311 factorizes the matrix \f$P\f$ and tries to find a factorized form of \f$P_t\f$ from a factorized form of \f$P_{t-1}\f$. Each of the \f$P\f$ matrices are Hermitian for the state and transition matrix provided above, so this matrix can actually be Cholesky factorized.
- This has nothing to do with the factorization of the \f$G Q G^\top\f$ term. In our code it is called a Cholesky factor GQ, *which it is not* in our case. In the case of a Wiener process, taking only acceleration as one that has noise, our \f$Q\f$ becomes a simple number \f$\sigma^2_a\f$, which forces the choice of \f$G\f$ (Section 6.3.3, page 274 of the Bar-Shalom book) and results in the following:
  \f[
    G \cdot Q \cdot G^\top = \left[\begin{matrix}\frac{dt^{2}}{2}\\dt\\1\end{matrix}\right] \cdot 
    \sigma^2_a \cdot \left[\begin{matrix}\frac{dt^{2}}{2} & dt & 1\end{matrix}\right] = 
    \left[\begin{matrix}\frac{dt^{4}}{4} & \frac{dt^{3}}{2} & \frac{dt^{2}}{2}\\\frac{dt^{3}}{2} & dt^{2} & dt\\\frac{dt^{2}}{2} & dt & 1\end{matrix}\right] \cdot \sigma^2_a
  \f]
  This matrix has a zero determinant and cannot be factorized, so the description for our `GQ_chol` variable is wrong as it is **not a Cholesky** factor. It is just such a matrix/vector that multiplied by itself transposed gives us some matrix that represents the process noise.

[1]: https://github.com/cra-ros-pkg/robot_localization/tree/dashing-devel
[2]: https://www.amazon.com/Estimation-Applications-Tracking-Navigation-Bar-Shalom/dp/047141655X
[3]: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/merge_requests/317#note_342833526
