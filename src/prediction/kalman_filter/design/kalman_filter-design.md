kalman_filter
========

# Purpose / Use cases

Autoware.Auto requires a method for state estimation. The main algorithms for state
estimation are particle filtering and kalman filtering. Of these two, Kalman
filtering was chosen for it's stronger theoretical guarantees and because it was
not dependent on random sampling.

Within the realm of kalman filtering, square root covariance filtering was
chosen for initial implementation.

This was done because this version of filtering has better numerical precision
and is much more robust to roundoff errors resulting in a non-symmetric positive
definite covariance matrix. The primary tradeoff is some (~50% more) extra
computational effort and a much more theoretically opaque implementation.

Within the subset of square root kalman filters, the Carlson-Schmidt square root
filter was chosen. This version was chosen because it keeps the interfaces
simple and the memory overhead somewhat less than U-D (i.e. Bierman-Thornton)
variants.

In most modern CPUs, square roots should not be overwhelmingly more expensive
than other floating point operations. Smidl and Peroutka's (see external links)
implementation shows similar performance between a UD filter and Carlson-Givens
implementation.

Between the two realizations for triangularization, the variant which uses
Givens rotations was used over the variant which uses Householder reflections.
This is because while Householder reflections typically have very good numerical
properties, the inner product term of the reflection can result in a loss of
precision. By contrast, Givens rotation has fairly bounded precision.

Finally, kalman filter implementations also lend themselves well to a probabilistic
interpretation, meaning they fit well into a multiple-model framework.


## Design

This implementation was intended to maximize numerical stability and minimize
computation and memory overhead.

Numerical stability is achieved by algorithm choice.

Lower computational requirements are achieved by some operation re-ordering,
caching, and scalar updates

Less memory overhead was achieved by pushing the burden of storing shared
matrices, such as measurement and process noise covariance, to an external
scope, rather than duplicating them over many instances of an EKF.

Finally, a note on convention:
We store the factored covariance matrices in lower triangular form.
The following factorization convention was used:
\f{aligned}{
P =& L L^{\top}\
=& U^{\top} U\
U =& L^{\top}
\f}
This implies that we do row trianglarization, that is:
\f{aligned}{
P' = A^{\top}A =& U^{\top}U + M_1^{\top}M_1\
=& \begin{bmatrix} U^{\top} & M_1^{\top} \end{bmatrix}
\begin{bmatrix} U \ M_1 \end{bmatrix} \
=& \begin{bmatrix} U^{\top} & M_1^{\top} \end{bmatrix} T^{\top}
T \begin{bmatrix} U \ M_1 \end{bmatrix} \
\f}
Where \f$T^{\top}T = I\f$, \f$T\f$ is a trianguarlization matrix.
Thus we set up our trianguarlization routine such that:
\f{aligned}{
A^{\top} =& \begin{bmatrix} L & M_1 \end{bmatrix} T^{\top}\
=& \begin{bmatrix} L' & 0 \end{bmatrix}
\f}
Lastly, we also expose some handles for IMM (interacting multiple models) mixing
in square root form. We accomplish this by decomposing the IMM covariance update:
\f{aligned}{
P'j =& \sum\limits{i=1}^r \mu_{j|i} \big(P_i + (x_i-\hat{x}_j)(x_i-\hat{x}j)^{\top}\big) \
=& \mu{j|j}\big(P_j + (x_j-\hat{x}j)(x_j-\hat{x}j)^{\top} \big) +
\sum\limits{i\neq j} \mu{j|i} \big(P_i + (x_i-\hat{x}_j)(x_i-\hat{x}_j)^{\top}\big) \
\f}
The first term then expresses itself as the method:
void imm_self_mix(const float32_t self_prob, const state_vec_t & x_mixed);
Which is assumed to be called first, as it updates the internal state vector.
Each element in the summation of the second term can then be expressed by the method:
void imm_other_mix(const float32_t other_prob, const state_vec_t & x_other, cov_mat_t & cov_other);
The update can be expressed in terms of cholesky factors:
\f{aligned}{
P' = L'L'^{\top} =& \sum\limits_{i=1}^r \mu_{j|i} \big(P_i + (x_i-\hat{x}j)(x_i-\hat{x}j)^{\top}\big) \
=& \begin{bmatrix} \cdots & \mu{j|i}L_i & \mu{j|i}(x_i - \hat{x}j) & \cdots\end{bmatrix}
\begin{bmatrix} \cdots & \mu{j|i}L_i & \mu_{j|i}(x_i - \hat{x}j) & \cdots\end{bmatrix}^{\top} \
=& \begin{bmatrix} \cdots & \mu{j|i}L_i & \mu_{j|i}(x_i - \hat{x}j) & \cdots\end{bmatrix}T T^{\top}
\begin{bmatrix} \cdots & \mu{j|i}L_i & \mu_{j|i}(x_i - \hat{x}j) & \cdots\end{bmatrix}^{\top} \
\begin{bmatrix}L' & 0 & \cdots & 0 \end{bmatrix} =&
\begin{bmatrix}  \cdots & \mu{j|i}L_i & \mu_{j|i}(x_i - \hat{x}_j) & \cdots\end{bmatrix} T
\f}
Meaning that the updates are just triangularization operations, similar to the temporal update.


### Assumptions / Known limits

Currently we assume decorrelated measurement noise (`R`), that is the matrix R
is diagonal. This is a simplification to keep logic simple for the filter.


### Inputs / Outputs / API

The configuration input to the kalman filter is as follows:

- A motion model, which has the following API:

```cpp
/// \brief Do prediction based on current state, store result somewhere else.
///        This is intended to be used with motion planning/collision avoidance
void predict(Eigen::Array<float32_t, NumStates> & x, const rclcpp::Duration & dt) = 0;
/// \brief Update current state with a given prediction. Note that this should be called
///        after compute_jacobian() as it will change the object's state. This is meant
///        to be called before doing assignment and observation updating. This is the
///        equivalent of temporal update for the state.
void predict(const rclcpp::Duration & dt) = 0;
/// \brief Compute the jacobian based on the current state and store the result somewhere else
void compute_jacobian(
Eigen::Matrix<float32_t, NumStates, NumStates> & F,
const rclcpp::Duration & dt) = 0;
/// \brief This is called by Esrcf. This should be first a computation of the jacobian, and
///        then a prediction to update the state. This is a distinct function because depending
///        on the motion model, there is some caching and optimization that can be done computing
///        both the prediction and jacobian together.
void compute_jacobian_and_predict(
Eigen::Matrix<float32_t, NumStates, NumStates> & F,
const rclcpp::Duration & dt) = 0;
/// \brief Get elements of the model's state.
float32_t operator[](const uint64_t idx) const = 0;
/// \brief Set the state
void set_state(const Eigen::Array<float32_t, NumStates> & x) = 0;
// TODO(c.ho) make this private for friends only?
Eigen::Array<float32_t, NumStates> & get_state();
```
  For an EKF, this model describes the nonlinear prediction function and the
  jacobian as a proxy for the transition model
- Matrices to describe the system:
    - `H` measurement matrix which maps from states to observations, can be
    provided on construction
    - `G` the matrix which maps from the process noise space to the state space,
    can (and should) be provided on construction
    - `Q` the process noise covariance matrix, can (and should) be provided on
    construction
    - `R` the measurement noise matrix, should be provided on construction and
    should be diagonal (e.g. white/uncorrelated noise)
- After construction, the primary method by which to interact with the model are
    - `temporal_update(dt)`: Performs prediction and covariance update. Note
    that `G` and/or the cholesky factor of `Q` can be provided as an input.
    - `observation_update(z, H)`: Performs the kalman fit step to update the
    state and covariace matrix
- Finally, after an observation update, the state and covariance matrix can be
extracted by `get_state()` in the motion model and `get_covariance()` on the
filter


## Inner-workings / Algorithm

The algorithms use Eigen operations.

Some modifications were made to the Householder triangularization. In
particular, the inner product of the row was cached instead of computing it
twice.

Modifications to Givens rotations were made to improve numerical stability and
reduce computational overhead:

- For some cases where a value is near zero, a multiply and add/subtract step is
dropped
    - See Bindel et. al. in the references section for more details on numerical
    stability improvements


### Error detection and handling

While the core algorithms rely on division and square roots, these operations
are safe so long as the measurement noise variance is positive (`R`). Only this
condition is checked on entry to the `scalar_update` function.

Similarly, for the temporal update (Householder version), the filter can guarantee
positive divisor if there are no fully zero rows in the composite matrix. This
will be checked at a higher level (above `Escrf`).

For the Givens version of the temporal update, the additional logic which
improves numerical stability and some performance also prevents singularities
which may occur due to division by 0.

`scalar_update()` has some potential for buffer overrun due to passing a
raw array. This is generally mitigated by mostly allowing access to this
function through a higher level function which has a typed measurement matrix as
input to prevent overrun. In the future, when row operations return blocks
instead of pointers, this should be even safer.


## Security considerations

TBD by a security specialist.


## References / External links

- Kalman Filtering Theory and Practice 3e - Grewal
- Estimation with Applications to Tracking and Estimation - Bar Shalom, Li
- [NASA JPL paper](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770005172.pdf)
- [Advantage of a Square Root Extended Kalman Filter for Sensorless Control of AC Drives](https://ieeexplore.ieee.org/abstract/document/6107581/)
- [On Computing Givens Rotations Reliably and Efficiently - Bindel et al](http://www.netlib.org/lapack/lawnspdf/lawn148.pdf)


## Future extensions / Unimplemented parts

- Interface cleanup due to math updates

Almost certainly will be implemented:

- Square root information filtering (SRIF)
- Measurement decorrelation

Possibly to-be implemented:

- Likelihood handle for SrcfCore
- Interacting Multiple Models (IMM)
- Parameter Learning
- Fixed lag smoothing
- Unscented Covariance/Information Filter
- Alternate core algorithms
  - UD filter (bierman-thornton)
  - Givens temporal update for carlson-schmidt
