How to Handle Errors at the Node Level {#error-handling}
========

@tableofcontents

# Goals {#error-handling-1}

This article describes how developers should handle errors in their nodes in Autoware.Auto.

# Introduction {#how-to-handle-errors-introduction}

Autoware.Auto currently uses basic error reporting through the ROS 2 logging mechanisms.

If you are not yet familiar with the concept of logging in ROS 2, see the [ROS 2 logging page](https://docs.ros.org/en/foxy/Concepts/About-Logging.html) for some examples and demos.

# Logging guidelines {#how-to-handle-errors-logging}

## Best practices {#how-to-handle-errors-best-practices}

Follow these best practices to ensure that your log messages are easily comprehensible.

- Put enough runtime context in the log message (including values of crucial variables) to help diagnose the issue (`WARN` and higher levels).

- When writing an `ERROR` or `FATAL` log message remember to also handle the situation programmatically.
  Return a meaningful value, throw an exception and handle it somewhere (if exceptions are allowed in your component), or transition to an erroneous state in the node's life cycle.
  Think about the system as a whole and what effects the situation might have beyond your node, and design the best way to adapt, exit, or repair and recover.

- When logging in the loop (meaning high frequency spin loops or callbacks of ROS nodes), use `RCLCPP_[SEVERITY]_THROTTLE` to control how often the message is logged (replace `[SEVERITY]` with `DEBUG`, `INFO` etc).
  This can be useful in a situation like logging the ego position not with the frequency of the updates, but in a more human-friendly way (e.g. only each second).
  If the log would be repetitive (e.g. the same `WARN` message with each pass), use the `RCLCPP_[SEVERITY]_ONCE` macro.

- Use logging output for automated testing, checking both the normal and erroneous execution.
  Log values that you can check to determine whether the behavior is as expected (e.g. results of node computations).

- Before submitting a merge request, check whether your new or changed log messages are following the guidelines.
  Eliminate unnecessary log messages used when developing a new feature.

# Log severity levels guide {#how-to-handle-errors-severity}

- `FATAL` – use this log level when a failure occurs that requires termination of the entire application.
  `FATAL` errors should only occur in non-optional components and are to be used sparingly.
  Example: a driver for a crucial piece of hardware fails to initiate or access the device.
  Note that it is sometimes difficult for a component programmer to determine how component failures affect the entire application – changes in log severity can be made after a review.

- `ERROR` – signals a serious issue with a component, either preventing it from working altogether and thus removing a part of functionality from the system or disabling some core functionality.
  Unlike `FATAL`, `ERROR` logs do not necessarily signal that the entire system went down abnormally.
  Examples: unable to record actuator/sensor data, unable to transform one of a sensor’s frames.

- `WARN` – signals something unusual or a problem which does not cause significant harm.
  It might also inform that a bigger problem is likely to occur in the future, e.g. a resource is running out.
  Warnings can also signal unusual delays, drop in quality or temporary lack of published data to which the component is subscribing.
  Warnings can escalate into errors, e.g. if delays become unacceptable.
  `WARN` is different from `INFO` in that it should be investigated.
  `WARN` logs should cover all such situations but show rarely at runtime in a well-developed application.

- `INFO` – informs that some notable (and expected) event occurred, such as a node transition to a distinct state, successful initiation of a component, or important service calls.
  Try to limit the amount of logging on this level and keep it concise and packed with useful information.

- `DEBUG` – is to be used for a more detailed information that can assist in debugging.
  The log output at the `DEBUG` level is expected to be quite comprehensive.
  It helps to find issues by improving visibility of the program flow.
  The output of `DEBUG` and `INFO` log levels should also be covered with automated testing.
  `DEBUG` level log messages can be used in loops, although developers should consider in each case whether it improves or degrades readability of the log. `_ONCE` and `_THROTTLE` variants of logging macros can be used to control the volume.

## C++ logging macros {#how-to-handle-errors-logging-macros}

Replace `[SEVERITY]` with `DEBUG`, `INFO` etc.

- `RCLCCP_[SEVERITY]` - default macro to use for logging with the use of format string (also accepts a single std::string argument).

- `RCLCPP_[SEVERITY]_ONCE` – use when you only want the log to be called once.
  Subsequent calls are ignored.
  This can be useful for warnings, when you expect to encounter the same situation with each pass.

- `RCLCPP_[SEVERITY]_EXPRESSION` – log a message when a condition is satisfied (convenient to skip the if clause).
  Note that it is still necessary to use the explicit if in case you want to do more than log on the condition.

- `RCLCPP_[SEVERITY]_FUNCTION` – log message only when a function returns false (useful with  “isOk()” type of function).

### New macros in Eloquent

- `RCLCPP_[SEVERITY]_STREAM` - default macro to use for using stream (<<) way of constructing the log.
  This is usually more convenient than using a format string. All other macros have stream variants since Eloquent (`_STREAM_EXPRESSION`, `_STREAM_ONCE`, `_STREAM_FUNCTION`).

- `RCLCPP_[SEVERITY]_THROTTLE` - log messages, but only as often as indicated.
  Useful for logging in high frequency loops and callbacks, when you care about how the value changes with time, but only need a less frequent update.

## Logging and automated testing {#how-to-handle-errors-automated-testing}

Use launch testing as described in [this article](https://github.com/ros2/launch/tree/master/launch_testing#launch_testing) to check whether the node outputs log messages as expected.
You can even extract values from logs and check them against constraints.

The presence of `FATAL` or `ERROR` logs should fail a normal test.
Presence of `FATAL` or `ERROR` logs that are different than expected should fail error handling tests.
Whether the presence of `WARN` logs should also fail the test can be determined during code review of a merge request.
