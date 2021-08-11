Noise model design {#noise-model-design}
=============

# Purpose / Use cases

One of the building blocks required for the Kalman Filter (see @ref state-estimation-design) is the
process noise model. In this document we cover the relevant documentation to the different ways on
how the noise can be modeled and which methods are used in this project.

## Design

All the noise models follow a Curiously Recurring Template Pattern (CRTP) interface defined in
`autoware::common::state_estimation::NoiseInterface`, which essentially allows using the following
functions with any noise model that follows this interface:

```cpp
auto covariance(const std::chrono::nanoseconds & dt) const;
```

As the interface is implemented following the CRTP pattern, all the classes that implement this
interface must implement a function `crtp_covariance` that returns the process noise covariance for
a given state vector. Internally, this function will be called from within the `NoiseInterface`
class.

Currently implemented concrete implementations:
- `autoware::common::state_estimation::UniformNoise` implements the Uniform noise model
- `autoware::common::state_estimation::WienerNoise` implements the Wiener noise model

Below are the details on their implementation.

### Uniform noise model

The class `autoware::common::state_estimation::UniformNoise` implements a simple model for the
uniform noise. It receives the variances for the different variables or a full covariance matrix and
stores this covariance internally, returning it (and scaling it by the elapsed time) when the
covariance is queried.

### Wiener noise model

The class `autoware::common::state_estimation::WienerNoise` implements a so-called Wiener noise
model (see
[here](https://nbviewer.jupyter.org/github/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb#Piecewise-White-Noise-Model)
for reference). The core idea behind it is that the user only specifies the noise on acceleration
and this noise gets propagated through the motion model down to the other lower-order variables.


## Error detection and handling
TBD


## Future extensions / Unimplemented parts

More motion models can be implemented if needed
