measurement_conversion {#measurement-conversion-design}
===========

This is the design document for the `measurement_conversion` package.


# Purpose / Use cases
This package provides conversions from message types to measurement types, i.e. types inheriting from `MeasurementInterface`. They contain values and their covariances.


# Design
The conversion is implemented as a function template. There is an additional template that takes a transform as input which is applied to a measurement.


## Assumptions / Known limits
Message objects are assumed to be valid, e.g. in a `PoseWithCovariance` message the covariance matrix has to be symmetric and positive semi-definite, quaternions have to be unit quaternions etc.


## Inputs / Outputs / API
As of writing, the API consists of two templated functions:

```
template<typename MeasurementT, typename MessageT>
MeasurementT message_to_measurement(const MessageT &)

template<typename MeasurementT>
MeasurementT transform_measurement(const MeasurementT &, const Eigen::Isometry3f &)
```

A footnote: the `transform_measurement` function also has another overload with two isometries to accomodate message types like `Odometry`.

Please search for this function in the API docs to see a list of available specializations.

For message and measurement types for which both templates are specialized, a combined function `message_to_transformed_measurement` is automatically available.


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
