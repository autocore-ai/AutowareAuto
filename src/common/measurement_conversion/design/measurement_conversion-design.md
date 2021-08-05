measurement_conversion {#measurement-conversion-design}
===========

This is the design document for the `measurement_conversion` package.


# Purpose / Use cases
This package provides conversions from message types to measurement types, i.e. types inheriting
from `MeasurementInterface`. They contain values and their covariances.


# Design
The conversion is implemented as a static templated function `from` of a templated class
`autoware::common::state_estimation::convert_to`. There is an additional templated function
`autoware::common::state_estimation::transform_measurement` that takes a transform as input which is
applied to a measurement.

The actual conversions would happen through specializing these classes and functions. The general
interface would always follow the lines:

```c++
convert_to<Stamped<PoseMeasurementXYZ64>>::from(msg);
```


## Assumptions / Known limits
Message objects are assumed to be valid, e.g. in a `PoseWithCovariance` message the covariance
matrix has to be symmetric and positive semi-definite, quaternions have to be unit quaternions etc.


## Inputs / Outputs / API
As of writing, the API consists of two templated approaches:

```
template<typename MeasurementT>
struct MEASUREMENT_CONVERSION_PUBLIC convert_to<Stamped<MeasurementT>>
{
  template<typename MsgT>
  static Stamped<MeasurementT> from(const MsgT & msg)
  {
    // The conversion code.
  }
};

template<typename MeasurementT>
MeasurementT transform_measurement(const MeasurementT &, const Eigen::Isometry3f &)
```

Please search for this function in the API docs to see a list of available specializations.

For message and measurement types for which both templates are specialized, a combined function
`message_to_transformed_measurement` is automatically available.


## Inner-workings / Algorithms
<!-- If applicable -->
N/A


## Error detection and handling
<!-- Required -->
N/A


# Security considerations
<!-- Required -->
N/A


# References / External links
<!-- Optional -->
N/A


# Future extensions / Unimplemented parts

Variants producing a double-precision measurement type could be added.


# Related issues
<!-- Required -->
N/A
