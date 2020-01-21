ndt_nodes
=============

# Purpose / Use cases

This package contains ndt related ROS2 nodes.

# Design

![ndt architecture](images/ndt_uml.svg)

## Map Publisher

[NDTMapPublisherNode](@ref autoware::localization::ndt_nodes::NDTMapPublisherNode) is responsible of reading a `.pcd` map from disk,
transforming this map into an ndt map and then publishing this map to its recipients.

Since the file IO means that this node cannot be used in a real time context, the dependency constraints are more relaxed and
 [pcl](https://github.com/PointCloudLibrary/pcl) is used for reading and writing of `.pcd` files.

### Algorithm Design
The workflow of the publisher can be summarized as the following:
1. Wait to discover recipients.
2. Use [pcl](https://github.com/PointCloudLibrary/pcl) to read a `.pcd` file into a `sensor_msgs::msg::PointCloud2` message.
3. Transform the read point cloud into a ndt map using [DynamicNDTMap](@ref autoware::localization::ndt::DynamicNDTMap).
4. Serialize the ndt map representation into a `PointCloud2` message where each point represents a single cell in the ndt map.
5. Publish the resulting PointCloud2 message.

The published point cloud message has the following fields:

```
name = "x",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "y",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "z",        count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_xx",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_xy",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_xz",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_yy",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_yz",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cov_zz",   count = 1U,     datatype = sensor_msgs::msg::PointField::FLOAT64
name = "cell_id",  count = 2U,     datatype = sensor_msgs::msg::PointField::UINT32
```

`cell_id` has a count of `2U` meaning it is practically an `unsigned int[2]`. This was necessary to represent 64 bit voxel indices since `sensor_msgs::msg::PointField` struct does not support 64 bit integer types.



# Related issues
- #136: Implement NDT Map Publisher