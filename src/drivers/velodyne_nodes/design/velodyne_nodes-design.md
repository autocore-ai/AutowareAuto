velodyne_nodes {#velodyne-nodes-design}
==============


# Purpose / Use cases

We require the Velodyne driver to be able to interface with a ROS-based system.


# Design

These nodes use the UdpDriver to receive the packets from the socket and then parse the packet into sensor_msgs::msg::PointCloud2 with the Vlp16Translator.

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


## Security considerations

These nodes inherit all security flaws and capabilities inherent in the UDP driver, Node,
and Vlp16Translator.

# Future extensions / Unimplemented parts

This node should eventually have health topic publishers and command subscribers
for fusion coordination.


# Related issues

- #4 - Implement velodyne driver
- #1309 - Update the driver for changes in ros udp driver 1.0

