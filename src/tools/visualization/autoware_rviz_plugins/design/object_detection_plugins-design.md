Autoware rviz plugins {#autoware-rviz-plugins}
=============

# Purpose / Use cases

Supporting plugins are needed to visualize the output of various packages inside autoware.auto

# Design

## BoundingBoxArrayDisplay

This class implements `rviz_common::RosTopicDisplay<autoware_auto_msgs::msg::BoundingBoxArray>`
to visualize BoundingBoxArray messages. It is a thin wrapper around
`rviz_default_plugins::displays::MarkerCommon` and in  general functions very similar to
`default_rviz_plugins::MarkerArrayDisplay`. Whenever a new `BoundingBoxArray` arrives, each
bounding box is converted to a marker and added to the internal display queue of
`rviz_default_plugins::displays::MarkerCommon`, which in return handles the markers in the
queue and their visualization.

## TrajectoryDisplay
This class implements `rviz_common::RosTopicDisplay<autoware_auto_msgs::msg::Trajectory>`
to visualize Trajectory messages. It is a thin wrapper around
`rviz_default_plugins::displays::MarkerCommon` and in  general functions very similar to
`default_rviz_plugins::MarkerArrayDisplay`. Whenever a new `Trajectory` arrives, each
trajectory point is converted to two markers, and added to the internal display queue of
`rviz_default_plugins::displays::MarkerCommon`, which in return handles the markers in the
queue and their visualization.
The two markers consists of `Arrow` for pose of the trajectory waypoint and `Text` for velocity of the same point.
You can also choose a color for the markers. Scale and Alpa value for both type of markers can be choosen independently.


# Related issues

- #152 - Create rviz2 plugins for displaying the BoundingBoxArray.msg