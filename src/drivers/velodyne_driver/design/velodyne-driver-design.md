velodyne_driver
===============

# Why we implemented this feature

The velodyne driver itself fundamentally converts packets to points. Communication with
UDP sockets, error handling and stateful transitions are handled by a different class,
namely autoware::drivers::udp_driver::UdpDriverNode.

Finally, care was taken to try to optimize the driver for speed as much as
possible. Expensive operations such as modulo and division were avoided in the
tightest loops, and as much work as possible was delegated to initialization
computation, and used during runtime via lookup tables. In addition, sleeps were
used in various loops to free up hardware resources when they might otherwise
not be needed.


# Modifications

One modification applied is that in addition to standard values, e.g. spatial
coordinates and intensity, we retain some order information from the sensor
itself.

Specifically, `fire_id` is kept track of, which consolidates points of a
point cloud into 16 point blocks that make up a fire pattern, or can be thought
of as a ray, where 16 in this case is the number of lasers in the particular
model of Velodyne LiDAR used. This information is used downstream, most
particularly in the `ray_ground_filter`.

An additional modification to the driver API is that points can be seen or
consumed before a full point cloud is received. This allows downstream
computation (e.g. filtering, ground segmentation) to happen as the point cloud
is coming in, to minimize the overall latency of the system. In this use case,
a special `fire_id`, `END_OF_SCAN_ID = 0xFFFF` is used to denote where one
pointcloud ends and another begins. In order to maintain consistency in a system
that works off these point streams, the point delimiting two pointclouds *must*
be pushed downstream.

The above stated point stream is distinct from a ROS/DDS topic in the following
ways:

- Point streams are intra-process, meaning they are sent between threads of the
same process
- Point streams are communicated via wait-free single producer single consumer
ring buffers, rather than tcp/udp
- Interface definitions have been defined (e.g. see `velodyne/point_stream.h`)
to allow different components to read from intermediate point streams or
directly from a point stream from another module (e.g. directly from the driver
  to the ray ground filter)
- Point streams can be converted into `PointCloud2` messages by accumulating
information into the message's data vector until `END_OF_SCAN_ID` is received
- `PointCloud2` messages can be converted into point streams by scanning through
messages upon receipt and pushing each point downstream

Another design goal for using point streams was to minimize exposure on the
ROS/DDS layer. If a DDS topic were used to communicate between threads instead,
there would be a huge amount of information being blasted across DDS at a very
high rate, even though other components don't need the information.


# Performance characterization


## Time

The running time of operating on any one packet and generating any one point is
`O(1)`. As the driver must wait for packets to come in from the sensor,
and because the number of packets in a full laser scan is user defined, the
runtime is `O(n)` in the size of the point cloud.


## Space

The space complexity for the Velodyne driver is `O(1)`. It is dominated by preallocated
lookup tables.


# Module IO (inputs/outputs) and constants

Inputs:

- Configuration parameters for `Vlp16Translator`

The velodyne driver itself generally converts to a vector of points to
message types.


# Reference

- [velodyne reference documentation](http://velodynelidar.com/downloads.html)
- [reference implementation for ROS1](https://github.com/ros-drivers/velodyne/tree/e306deb03c033c0f3d41c27b8b7b4979251eb1fa)


# Related issues

- #4 - Implement velodyne driver
