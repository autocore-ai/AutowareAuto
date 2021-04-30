measurement_conversion {#measurement-conversion-design}
===========

This is the design document for the `measurement_conversion` package.


# Purpose / Use cases
<!-- Required -->
This package provides conversions from message types to measurement types, i.e. types inheriting from `MeasurementInterface`.

# Design
<!-- Required -->
The conversion is implemented as a function template. In addition to the message, it additionally takes a transform as input which is applied to the measurement.


## Assumptions / Known limits
Message objects are assumed to be valid, e.g. in a `PoseWithCovariance` message the covariance matrix has to be symmetric and positive semi-definite, quaternions have to be unit quaternions etc.


## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
As of writing, the API consists of a templated function `message_to_measurement` with two overloads:

```
template<typename MeasurementT, typename MessageT>
MeasurementT message_to_measurement(const MessageT &, const Eigen::Isometry3f &)

template<typename MeasurementT, typename MessageT>
MeasurementT message_to_measurement(
  const MessageT &, const Eigen::Isometry3f &, const Eigen::Isometry3f &)
```

Please search for this function in the API docs to see a list of available specializations.


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
At the time of writing, the measurement types capture only a subset of the information conveyed by the message types, e.g. only the `x` and `y` coordinates are extracted from a `geometry_msgs::msg::Pose`, but not `z` or any orientation. More comprehensive measurement types could be added.

Variants producing a double-precision measurement type could be added.


# Related issues
<!-- Required -->
N/A
