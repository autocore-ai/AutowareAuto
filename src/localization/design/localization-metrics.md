Localization metrics {#localization-metrics}
===========================================

# Introduction

Concrete metrics (i.e. those that generate numeric values) are important for the general development
of software components and the development of algorithms.

Metrics allow for the definition of acceptance criteria for general software components, and provide
a common language for the comparison between implementations of some standard, in this case
localization algorithms.

# Metrics

The metrics are broken into four categories:

1. Software quality metrics
2. Embedded software metrics
3. General algorithm metrics
4. Localization algorithm metrics

Two general caveats should be mentioned:

1. Where requirements on metrics come from should be understood. Requirements may not be universally
true or correct
2. For all metrics other than software quality metrics, the metric value is use-case specific,
meaning a specified value is only valid for a given environment and hardware (i.e. sensors, compute)
setup

## Software quality metrics

The following software quality metrics are defined:

1. Testing coverage (in order of increasing strictness):
    1. Line coverage
    2. Function coverage
    3. Branch coverage
    4. MC/DC coverage
    5. Basis path coverage
2. General
[software quality metrics](https://www.hanselman.com/blog/content/binary/NDepend%20metrics%20placemats%201.1.pdf),
such as those covered by the [CppDepend tool](https://www.cppdepend.com/metrics)
3. Lines of code
4. Number of external project dependencies

Two caveats should be mentioned:

1. Testing coverage is only meaningful when the test is meaningful. Random API calls to achieve
testing coverage is not as valuable as tests driven by requirements and edge cases
2. Software quality metrics are useful for smell tests, but are not replacements for human review
and design. These metrics can define recommendations, but not hard and fast rules

## Embedded software metrics

The following metrics are defined for general embedded software:

1. Binary size
2. Maximum memory usage
3. Average CPU load for a given use case (inputs, hardware, OS)

## General algorithm metrics

The following metrics are defined:

1. Mean/max/95% algorithm runtime
2. Mean/max/95% algorithm iterations
3. Mean/max/95% algorithm runtime/iteration

## Localization algorithm metrics

The following localization metrics are defined:

1. Mean/max/min/95% translation error (meters)
2. Mean/max/min/95% rotation error (radians)
3. Success rate as a function of error in initial guess
4. Minimum input density/size

Further non-qualitative metrics can also be defined:
1. Requirements on inputs
2. Guarantees (e.g. optimality, convergence, etc.)
