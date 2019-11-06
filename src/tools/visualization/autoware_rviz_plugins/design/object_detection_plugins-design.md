autoware_rviz_plugins
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

# Related issues

- #152 - Create rviz2 plugins for displaying the BoundingBoxArray.msg