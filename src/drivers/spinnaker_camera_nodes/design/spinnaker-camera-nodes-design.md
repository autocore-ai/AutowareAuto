Spinnaker camera node {#spinnaker-camera-node}
=====================

# Purpose / Use cases
We need a node that would use the Spinnaker camera driver to connect to Pointgrey/FLIR cameras and publish their images as ROS 2 messages.

# Design
We inherit from the `rclcpp::Node` and link the `publish_image` function as a callback to a Spinnaker SDK wrapper.

## Inputs / Outputs / API
The input is the images generated from the cameras and read by the driver. The driver will then make them available to this node.

## Configuration

The node is configured through the parameters. 
Here is a short recap of the most important parts.
For more details, see the `spinnaker_camera_node.param.template.yaml` file.

It can be used to run the node as follows:

```bash
ros2 run spinnaker_camera_nodes spinnaker_camera_node_main --ros-args --params-file ./install/spinnaker_camera_nodes/share/spinnaker_camera_nodes/param/spinnaker_camera_node.param.template.yaml
```

### Configuring cameras
It supports two ways of configuration:
- providing a single instance of camera settings that will be used for all cameras.
- providing an instance of camera settings per camera. The number of these must match the number of available cameras.

### Configuring publishers
If `one_publisher_per_camera` is set to `true`, there is going to be as many publishers as there are cameras, publishing on different topics. If set to `false` a single publisher with a single topic will be reused. The messages can be then discriminated on the basis of their `frame_id`. 

# Related issues

- #395 - Implement ROS 2 node for Spinnaker driver
