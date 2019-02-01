motion_model
========

# Purpose / Use cases

Autoware.Auto require a means to abstract the process model from the
measurement/uncertainty models for state estimation. This allows a generic state
estimation (e.g. kalman filter) implementation to be used with many different
process models.


## Constant velocity

A constant velocity motion model was used to provide a perfect linear model for the purpose of
testing the state estimation schemes.


## Constant acceleration

This motion model is a perfect linear model that also can provide curved paths.
In fact, the CATR model (below) can be recovered from the constant acceleration
parameters.
The linearity, expressive power, and cheaper computation make the constant acceleration
model attractive compared to the CATR model.


## Constant Acceleration and Turn Rate (CATR)

The CATR model was chosen to provide a minimally descriptive kinematic model for how cars move. This
represents a zero-order hold on vehicle controls. Constant acceleration roughly corresponds to a
fixed throttle. Constant turn rate roughly corresponds to fixed wheel angle.

This enables minimal kinematic reasoning for somewhat more complex maneuvers, such as curves, and
keep the models valid for a slightly longer time horizon.

In addition, this model was shown to have lower error compared to other models. For more details,
see Schubert et. al. in the reference section.


## Parameter estimator

The parameter estimator model is a simple model that provides capabilities to do parameter
estimation for a set of potential correlated parameters.

In the context of our motion models, it is intended to provide an extension to the CATR model by
having decoupled parameter estimates of the bounding box size.


## Design

The primary purpose of the motion model class is to:

- Wrap a state vector
- Provide a means to predict/update the state vector according to the model
- Provide computation of the process jacobian

The following considerations were included for the API:

- The motion model should have ownership over it's state
    - This is why there only const indexing and `set_internal_state(x)`
- A non-mutating prediction function exists to provide an interface by which you
can do prediction rollouts
- `compute_jacobian_and_predict()` was added because there may be some common
computation between jacobian computation and prediction for a given model.


### Assumptions / Known limits

The interface is designed as is. It is unclear how it will change and if it can  be extended to
motion models for planning.

Particular models have various assumptions baked in.


#### Constant velocity

The constant velocity instantiation of the model is purely linear and assumes constant velocity.


#### Constant Acceleration and Turn Rate (CATR)

The CATR instantiation of the model assumes constant turn rate and constant acceleration.

Turn rate is assumed to be independent of all other parameters, such as velocity or acceleration.

This is a nonlinear model.


#### Parameter estimator

This model assumes no dynamics.


### Inputs / Outputs / API

The following inputs are provided:

- State (i.e. `set_internal_state()`)
- Time increment (i.e. `predict(), compute_jacobian()`)

The following outputs are provided:

- State (`get_state()`, `operator[]`)
- Jacobian (`compute_jacobian(F)`)


### Inner-workings / Algorithms

The motion model class is primarily an interface.


#### Constant velocity

The constant velocity motion model has the following state:

```c
x  // x position
y  // y position
u  // x velocity
v  // y velocity
```

With the following update equation:

```c
x = x + u * dt
y = y + v * dt
u = u
v = v
```

Resulting in the following jacobian:

```c
1  0  dt 0
0  1  0  dt
0  0  1  0
0  0  0  1
```


#### Constant acceleration

The constant acceleration model has the following state:

```c
x   // position terms
y
vx  // velocity terms
vy
ax  // acceleration terms
ay
```

And the following update equation:

```c
x = x + vx * dt + 0.5 * dt * dt * ax
y = y + vy * dt + 0.5 * dt * dt * ay
vx = vx + dt * ax
vy = vy + dt * ay
```

Resulting in the following jacobian:

```c
1  0  dt 0  0.5*dt*dt 0
0  1  0  dt 0         0.5*dt*dt
0  0  1  0  dt        0
0  0  0  1  0         dt

Similarly, the velocity and acceleration magnitude, heading and turn rate
can be expressed by the following equations:

```c
v = sqrtf((vx * vx) + (vy * vy));
a = sqrtf((ax * ax) + (ay * ay));
th = atan2f(vy, vx);
th_dot = (vx * ay - vy * ax) / v;
```

The derivations are left as an exercise for the reader.


#### Constant acceleration and turn rate

Run the script contained in `motion_models/scripts/catr_diff.py` for a computation of the
derivatives and formula for position update


#### Parameter estimator

The parameter estimator is templated for `N` parameters. The process model is
then the identity matrix:

```c
1 0 0
0 1 0
0 0 1
```

For `N=3` parameters.


## Security considerations

TBD by a security specialist.


## References / External links

- [Comparison and Evaluation of Advanced Motion Models for Vehicle Tracking, Schubert et al](https://pdfs.semanticscholar.org/5dd9/709902c328c8f8cc8aa0d02ce2f23dac41c7.pdf)


## Future extensions / Unimplemented parts

N/A
