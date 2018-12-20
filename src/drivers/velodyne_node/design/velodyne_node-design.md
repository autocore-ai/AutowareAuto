velodyne_node
=============


# Purpose / Use cases

We require the Velodyne driver to be able to interface with a ROS-based system.


# Design

These nodes are specializations of the UdpDriver, composed with the Vlp16Translator.

Specifically, these specializations are for the sensor_msgs::msg::PointCloud2
message type which is intended for widespread use and visualization, but not in high performance
or safety-critical settings.

The purpose of these nodes are to convert Udp packets from a VLP16 HiRes sensor into
ROS 2 messages.


## Assumptions / Known limits

Assumes input on some UDP port from a Velodyne sensor.

For the VelodyneCloudNode, it is assumed that the PointCloud2 message is at least larger
than velodyne_driver::Vlp16Translator::POINT_BLOCK_CAPACITY, which is 512.


## Inputs / Outputs / API

Input:

- UDP packets from a VLP-16 LiDAR sensor

Output:

- PointCloud2 message


## Error detection and handling

These nodes are built on top of a UdpDriver, which itself is built off a LifecycleNode, and thus
inherits their error handling capabilities.

In addition to any other new features to be added to the base classes.

## Security considerations

These nodes inherit all security flaws and capabilities inherent in the UDP driver, LifecycleNode,
and Vlp16Translator.

# Future extensions / Unimplemented parts

This node should eventually have health topic publishers and command subscribers
for fusion coordination.


# Related issues

- #4 - Implement velodyne driver

