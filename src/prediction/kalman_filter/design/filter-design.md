Filtering {#filter-design}
=============

# Purpose / Use cases

To smooth incoming measurements it is common to use a probabilistic filter. There are different
flavors of these. Just to name a few common options, there are (Extended) Kalman Filter, Unscented
Kalman Filter or Particle filter. Each of these comes with their advantages and disadvantages. In
this package we implement a generic filter interface (`autoware::prediction::FilterInterface`) and a
number of concrete implementations for it.

Currently implemented filters are:

- `autoware::prediction::KalmanFilter` that can be used both as a standard Kalman Filter and as
  an Extended Kalman Filter.


## Design

The cornerstone of the design is the `autoware::prediction::FilterInterface`, which defines a number
of functions that the concrete implementations of this filter can implement. The interface is
templated, and thus follows the Curiously Recurring Template Pattern (CRTP) pattern, to allow for
using concrete types and to avoid requiring pointers in order to make filter implementations
polymorphic.

In its essence, every filter that implements the proposed interface will have a number of functions:

```cpp
  auto predict(const std::chrono::nanoseconds & dt);

  template<typename MeasurementT>
  auto correct(const MeasurementT & measurement);

  template<typename StateT>
  void reset(const StateT & state, const typename StateT::Matrix & covariance);

  auto & state();
  const auto & state() const;

  auto & covariance();
  const auto & covariance() const;
```

Because the interface follows the CRTP paradigm, the classes that implement this interface do not
implement functions like `correct` and `predict` directly, but rather internal functions (here, with
`crtp_` prefix) that get called by the interface.

It is expected here that `MeasurementT` class implements the
`autoware::prediction::MeasurementInterface` class, while `StateT` class is a specialization of the
`autoware::prediction::GenericState` class.

### (Extended) Kalman filter design

The class `autoware::prediction::KalmanFilter` declared in `kalman_filter.hpp` implements the
`autoware::prediction::FilterInterface`.

In order to create an instance of the Kalman filter class the user must provide instances of
`autoware::prediction::MotionModelInterface` and `autoware::prediction::NoiseInterface` classes.
These are then used in the implementation of the `crtp_predict` and `crtp_correct` functions.

@warning until issue #944 is closed the angles are not wrapped in the state.

#### Extended Kalman filter algorithm

Formally, the Kalman filter implementation in this package uses the following algorithm. Let
\f$x\f$ represent the current state of some object. In addition to this, a matrix \f$S\f$ is the
covariance matrix measuring the uncertainty of this state. Furthermore, let \f$f\f$ be a non-linear
transition function with its Jacobian being \f$F\f$ and uncertainty measured by a matrix \f$Q\f$.
Similar logic holds for a measurement function \f$h\f$ that observes state \f$x\f$ in a potentially
non-linear fashion. In other words, this function, given an unknown state \f$\hat{x}\f$ produces a
measurement from this state. This function likewise has a Jacobian, denoted here with \f$H\f$.

Kalman filter iteratively repeats the following steps:
- predict the current state and its covariance forward in time
- given a noisy observation of the environment it estimates by how much its prediction must be corrected
- a corrected state and covariance are computed

More formally, with a vector \f$m\f$ denoting a real-world observation, and \f$R\f$ denoting its
covariance:

\f{aligned}{
x_\mathrm{predicted} &= f(x) \\
S_\mathrm{predicted} &= F S F^\top + Q \\
\\
i &= m - h(x_\mathrm{predicted}) \\
S_i &= H S_\mathrm{predicted} H^\top + R \\
K &= S_\mathrm{predicted} H^\top S_i^{-1} \\
\\
x_\mathrm{corrected} &= x_\mathrm{predicted} + K i \\
S_\mathrm{corrected} &= (I - K H) S_\mathrm{predicted}, \\
\f}

here, \f$I\f$ is the identity matrix, \f$i\f$ is the "innovation" (intuitively: how far is the
predicted measurement from an observed one), \f$S_i\f$ is the innovation covariance. The core
components that helps "decide" how to correct the state and intuitively "weights" the certainty in
the current state predictions versus the measurement certainty is the "Kalman gain", denoted here as
\f$K\f$.


## Error detection and handling

The interface is static and thus most of the errors can be caught at compile time. Sensible
`static_assert`s are scattered throughout the code to catch a programming error early and to provide
a meaningful compilation error.


## Future extensions / Unimplemented parts

The `autoware::prediction::FilterInterface` is generic enough to implement different _kinds_ of
filters. The ones that come to mind are:
- Unscented Kalman Filter
- Square Root covariance filter

This is planned to be done in the future if needed.


## Related issues:
- [`#865`](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/865): Redesign
  kalman filter class hierarchy
