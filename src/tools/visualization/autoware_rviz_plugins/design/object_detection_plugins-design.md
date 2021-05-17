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

## ObjectPolygonDisplayBase  
This is an abstract class that helps in creating plugins for types that contain `autoware_auto_msgs::msg::Shape` and `autoware_auto_msgs:msg::ObjectClassification` fields. This base class creates properties for plugins that let the color and transparency values for shapes to be customized based on the class label. It also creates a property to toggle on or off the 3d visualization of the shape. Apart from the properties, this class also has a `get_marker_ptr` function that produces a `visualization_msgs::msg::Marker::SharedPtr` for the provided shape and classification label.  
The child classes deriving from this class have to define `processMessage` function which will act as the callback for the message for which the plugin is being implemented.   

### DetectedObjectsDisplayBase  
This class derives from `ObjectPolygonDisplayBase` to implement `RosTopicDisplay` for `autoware_auto_msgs::msg::DetectedObjects`. It visualizes the shape field as a convex 2D or 3D polygon and colors it according to the class label in the msg.  

### TrackedObjectsDisplayBase  
This class derives from `ObjectPolygonDisplayBase` to implement `RosTopicDisplay` for `autoware_auto_msgs::msg::TrackedObjects`. It visualizes the shape field as a convex 2D or 3D polygon and colors it according to the class label in the msg. This class also creates text marker to visualize the `id` of the objects in the msg.  

# Related issues

- #152 - Create rviz2 plugins for displaying the BoundingBoxArray.msg
- #900 - Implement rviz2 plugin for TrackedObjects.msg
- #1110 - Implement rviz2 plugin for DetectedObjects.msg
