Motion Model {#motion-model-design}
=============

## Motivation
In order to predict the movement of any object forward a notion of a motion model is required. In
this package we provide an interface and concrete implementations for the different motion models,
both linear and non-linear ones.

## Proposed design

All motion models implement `autoware::prediction::MotionModelInterface`, a
[CRTP](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern) interface defined in
`motion_model_interface.hpp` file, i.e., they are expected to implement the following functions:

```cpp
// Returns the state forward predicted by the given time increment
template<typename StateT>
auto crtp_predict(const StateT & state, const std::chrono::nanoseconds & dt) const;
// Returns a Jacobian of the motion model. In the linear case, that is equal to a transition matrix.
template<typename StateT>
auto crtp_jacobian(const StateT & state, const std::chrono::nanoseconds & dt) const;
```

A motion model that conforms to this interface can then be called as:

```cpp
new_state = motion_model.predict(state, dt);
jacobian = motion_model.jacobian(state, dt);
```

### Assumptions / Known limits
There can be multiple implementations for the motion model, for example, a linear one or a
differential drive one. These are implementations of the proposed
`autoware::prediction::MotionModelInterface`. Some of these implementation can be general, while
other must be tailored to a particular state. Generally speaking, more complex, non-linear models
would need to be specialized for a specific state.

## Inner-workings / Algorithm

### Stationary Motion Model
A stationary motion model implemented in `autoware::common::motion_model::StationaryMotionModel`
class is a motion model that ensures that the state remains stationary, i.e., there is no change to
the state.

The Jacobian of this motion model is therefore a simple identity matrix of the size matching the
state vector dimensionality.

### Linear Motion Model
A linear motion model implemented in `autoware::common::motion_model::LinearMotionModel` class is a motion
model that expects that all dimensions (e.g., variables `X`, `Y`, `Z`, see `common_variables.hpp`)
are independent, thus the transition function of such a model can be represented by matrix
multiplication. Therefore, prediction happens following the formula \f$s_1 = J s_0\f$.

The Jacobian of this motion model is specialized for each state. A utility function that constructs
a block matrix for position, speed and acceleration variables given a \f$\Delta t\f$ can be used if
the variables of the given state are present in a row, e.g., `FloatState<X, X_SPEED, X_ACCELERATION,
Y, Y_SPEED, Y_ACCELERATION>`. An example of a transition matrix for such a state would look as
follows:

\f{aligned}{
J_\mathrm{linear} = \left[\begin{matrix}1 & \Delta{t} & \frac{\Delta{t}^{2}}{2} & 0 & 0 & 0\\0 & 1 & \Delta{t} & 0 & 0 & 0\\0 & 0 & 1 & 0 & 0 & 0\\0 & 0 & 0 & 1 & \Delta{t} & \frac{\Delta{t}^{2}}{2}\\0 & 0 & 0 & 0 & 1 & \Delta{t}\\0 & 0 & 0 & 0 & 0 & 1\end{matrix}\right]
\f}

### Differential Drive Motion Model

A differential drive motion model models an object moving in the direction in which it is facing, as
opposed to freely in two dimensions. This direction (the yaw) is variable. This is a non-linear
motion model, and two specific variants are provided: a CVTR (Constant Velocity and Turn Rate) and a
CATR (Constant Acceleration and Turn Rate) model.

Derivations for these models can be found in the [motion_model.ipynb](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/common/motion_model/notebooks/motion_model.ipynb) file.

#### CATR (Constant Acceleration and Turn Rate)

See `autoware::common::motion_model::CatrMotionModel` typedef for details.

This model assumes a state:

\f{aligned}{
s = \left[ x, y, \theta, v, \omega, a \right]^\top,
\f}
where \f$x\f$ and \f$y\f$ are the 2D coordinates of an object, \f$\theta\f$ is its yaw (orientation,
measured counterclockwise from the X axis), \f$v\f$ is the linear velocity in the direction in which
the object is facing, \f$\omega\f$ is angular velocity (aka turn rate), and \f$a\f$ is the linear
acceleration along the orientation direction.

Assuming constant \f$\omega\f$ and \f$a\f$, the next state can be rewritten as follows:

\f{aligned}{
s_{\mathrm{next}} = \left[\begin{matrix}x\\y\\\theta\\v\\\omega\\a\end{matrix}\right] + \int_0^{\Delta t} \left[\begin{matrix} (v + \int_0^{\Delta t} a, dt) \cos{\left(\theta + \int_0^{\Delta t} \omega, dt \right)}\\ (v + \int_0^{\Delta t} a, dt) \sin{\left(\theta + \int_0^{\Delta t} \omega, dt \right)}\\ \omega\\ a\\0\\0\end{matrix}\right], dt
\f}

Putting this into SymPy, the predicted state can be computed as follows:

\f{aligned}{
s_{\mathrm{next}} = \left[\begin{matrix}x + \begin{cases} \frac{\Delta{t} a \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{v \sin{\left(\theta \right)}}{\omega} + \frac{v \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{a \cos{\left(\theta \right)}}{\omega^{2}} + \frac{a \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}} & \text{for}\: \omega > -\infty \wedge \omega < \infty \wedge \omega \neq 0 \\\left(\frac{\Delta{t}^{2} a}{2} + \Delta{t} v\right) \cos{\left(\theta \right)} & \text{otherwise} \end{cases}\\y + \begin{cases} - \frac{\Delta{t} a \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} + \frac{v \cos{\left(\theta \right)}}{\omega} - \frac{v \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{a \sin{\left(\theta \right)}}{\omega^{2}} + \frac{a \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}} & \text{for}\: \omega > -\infty \wedge \omega < \infty \wedge \omega \neq 0 \\\left(\frac{\Delta{t}^{2} a}{2} + \Delta{t} v\right) \sin{\left(\theta \right)} & \text{otherwise} \end{cases}\\\Delta{t} \omega + \theta\\\Delta{t} a + v\\\omega\\a\end{matrix}\right]
\f}

Note that there is a difference for the cases when \f$\omega\f$ is 0 and when it is not.

In the non-zero case, the Jacobian will be:

\f{aligned}{
J_{\omega \ne 0} = \left[\begin{matrix}1 & 0 & \frac{\Delta{t} a \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{v \cos{\left(\theta \right)}}{\omega} + \frac{v \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} + \frac{a \sin{\left(\theta \right)}}{\omega^{2}} - \frac{a \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}} & - \frac{\sin{\left(\theta \right)}}{\omega} + \frac{\sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} & \frac{\Delta{t}^{2} a \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} + \frac{\Delta{t} v \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{2 \Delta{t} a \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}} + \frac{v \sin{\left(\theta \right)}}{\omega^{2}} - \frac{v \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}} + \frac{2 a \cos{\left(\theta \right)}}{\omega^{3}} - \frac{2 a \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega^{3}} & \frac{\Delta{t} \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{\cos{\left(\theta \right)}}{\omega^{2}} + \frac{\cos{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}}\\0 & 1 & \frac{\Delta{t} a \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{v \sin{\left(\theta \right)}}{\omega} + \frac{v \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{a \cos{\left(\theta \right)}}{\omega^{2}} + \frac{a \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}} & \frac{\cos{\left(\theta \right)}}{\omega} - \frac{\cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} & \frac{\Delta{t}^{2} a \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} + \frac{\Delta{t} v \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} + \frac{2 \Delta{t} a \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}} - \frac{v \cos{\left(\theta \right)}}{\omega^{2}} + \frac{v \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}} + \frac{2 a \sin{\left(\theta \right)}}{\omega^{3}} - \frac{2 a \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega^{3}} & - \frac{\Delta{t} \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{\sin{\left(\theta \right)}}{\omega^{2}} + \frac{\sin{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}}\\0 & 0 & 1 & 0 & \Delta{t} & 0\\0 & 0 & 0 & 1 & 0 & \Delta{t}\\0 & 0 & 0 & 0 & 1 & 0\\0 & 0 & 0 & 0 & 0 & 1\end{matrix}\right],
\f}
while if \f$\omega\f$ _is_ zero it reduces to the following form:

\f{aligned}{
J_{\omega = 0} = \left[\begin{matrix}1 & 0 & - \frac{\Delta{t}^{2} a \sin{\theta}}{2} - \Delta{t} v \sin{\theta} & \Delta{t} \cos{\theta} & - \frac{\Delta{t}^{3} a \sin{\theta}}{2} - \Delta{t}^{2} v \sin{\theta} & \frac{\Delta{t}^{2} \cos{\theta}}{2}\\0 & 1 & \frac{\Delta{t}^{2} a \cos{\theta}}{2} + \Delta{t} v \cos{\theta} & \Delta{t} \sin{\theta} & \frac{\Delta{t}^{3} a \cos{\theta}}{2} + \Delta{t}^{2} v \cos{\theta} & \frac{\Delta{t}^{2} \sin{\theta}}{2}\\0 & 0 & 1 & 0 & 0 & 0\\0 & 0 & 0 & 1 & 0 & \Delta{t}\\0 & 0 & 0 & 0 & 1 & 0\\0 & 0 & 0 & 0 & 0 & 1\end{matrix}\right]
\f}

#### CVTR (Constant Velocity and Turn Rate)

See `autoware::common::motion_model::CvtrMotionModel` typedef for details.

This motion model is very similar to the CATR one, just a little simpler. It assumes the state:
\f{aligned}{
s = \left[ x, y, \theta, v, \omega\right]^\top,
\f}
where \f$x\f$ and \f$y\f$ are the 2D coordinates of an object, \f$\theta\f$ is its yaw (orientation,
measured counterclockwise from the X axis), \f$v\f$ is the linear velocity in the direction in which
the object is facing, and \f$\omega\f$ is angular velocity (aka turn rate).

Assuming constant \f$\omega\f$ and \f$v\f$, the next state can be rewritten as follows:
\f{aligned}{
s_{\mathrm{next}} = \left[\begin{matrix}x\\y\\\theta\\v\\\omega\end{matrix}\right] + \int_0^{\Delta t} \left[\begin{matrix} v \cos{\left(\theta + \int_0^{\Delta t} \omega, dt \right)}\\ v \sin{\left(\theta + \int_0^{\Delta t} \omega, dt \right)}\\ \omega\\0\\0\end{matrix}\right], dt
\f}

Same as before, plugging this into SymPy, we get:

\f{aligned}{
s_{\mathrm{next}} = \left[\begin{matrix}x + \begin{cases} - \frac{v \sin{\left(\theta \right)}}{\omega} + \frac{v \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} & \text{for}\: \omega > -\infty \wedge \omega < \infty \wedge \omega \neq 0 \\\Delta{t} v \cos{\left(\theta \right)} & \text{otherwise} \end{cases}\\y + \begin{cases} \frac{v \cos{\left(\theta \right)}}{\omega} - \frac{v \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} & \text{for}\: \omega > -\infty \wedge \omega < \infty \wedge \omega \neq 0 \\\Delta{t} v \sin{\left(\theta \right)} & \text{otherwise} \end{cases}\\\Delta{t} \omega + \theta\\v\\\omega\end{matrix}\right]
\f}

Taking the Jacobian of \f$s_{\mathrm{next}}\f$ with respect to \f$s\f$ yields:

\f{aligned}{
J_{\omega \ne 0} &= \left[\begin{matrix}1 & 0 & - \frac{v \cos{\left(\theta \right)}}{\omega} + \frac{v \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} & - \frac{\sin{\left(\theta \right)}}{\omega} + \frac{\sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} & \frac{\Delta{t} v \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} + \frac{v \sin{\left(\theta \right)}}{\omega^{2}} - \frac{v \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}}\\0 & 1 & - \frac{v \sin{\left(\theta \right)}}{\omega} + \frac{v \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} & \frac{\cos{\left(\theta \right)}}{\omega} - \frac{\cos{\left(\Delta{t} \omega + \theta \right)}}{\omega} & \frac{\Delta{t} v \sin{\left(\Delta{t} \omega + \theta \right)}}{\omega} - \frac{v \cos{\left(\theta \right)}}{\omega^{2}} + \frac{v \cos{\left(\Delta{t} \omega + \theta \right)}}{\omega^{2}}\\0 & 0 & 1 & 0 & \Delta{t}\\0 & 0 & 0 & 1 & 0\\0 & 0 & 0 & 0 & 1\end{matrix}\right]\\
\\
J_{\omega = 0} &= \left[\begin{matrix}1 & 0 & - \Delta{t} v \sin{\theta} & \Delta{t} \cos{\theta} & - \Delta{t}^{2} v \sin{\theta}\\0 & 1 & \Delta{t} v \cos{\theta} & \Delta{t} \sin{\theta} & \Delta{t}^{2} v \cos{\theta}\\0 & 0 & 1 & 0 & 0\\0 & 0 & 0 & 1 & 0\\0 & 0 & 0 & 0 & 1\end{matrix}\right]
\f}



# References

[`#865`](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/865) - Redesign kalman filter class hierarchy

@subpage motion-model-notebook-readme
