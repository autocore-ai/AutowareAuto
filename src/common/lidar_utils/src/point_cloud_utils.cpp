// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

//lint -e537 pclint vs cpplint NOLINT
#include <string>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "lidar_utils/lidar_types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"

namespace autoware
{
namespace common
{
namespace lidar_utils
{

////
void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size)
{
  msg.height = 1U;
  msg.fields.resize(4U);
  msg.fields[0U].name = "x";
  msg.fields[1U].name = "y";
  msg.fields[2U].name = "z";
  msg.fields[3U].name = "intensity";
  msg.point_step = 0U;
  for (uint32_t idx = 0U; idx < 4U; ++idx) {
    msg.fields[idx].offset = static_cast<uint32_t>(idx * sizeof(float));
    msg.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[idx].count = 1U;
    msg.point_step += static_cast<uint32_t>(sizeof(float));
  }
  const std::size_t capacity = msg.point_step * size;
  msg.data.reserve(capacity);
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.header.frame_id = frame_id;
}

////////////////////////////////////////////////////////////////////////////////

bool add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::lidar_utils::PointXYZIF & pt,
  int & point_cloud_idx)
{
  bool ret = false;

  sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z_it(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity_it(cloud, "intensity");

  x_it += point_cloud_idx;
  y_it += point_cloud_idx;
  z_it += point_cloud_idx;
  intensity_it += point_cloud_idx;

  // Actual size is 20 due to padding by compilers for the memory alignment boundary.
  // This check is to make sure that when we do a insert of 16 bytes, we will not stride
  // past the bounds of the structure.
  static_assert(
    sizeof(autoware::common::lidar_utils::PointXYZIF) >= ((4U * sizeof(float)) + sizeof(uint16_t)),
    "PointXYZIF is not expected size: ");

  if (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end() &&
    intensity_it != intensity_it.end())
  {
    // add the point data
    *x_it = pt.x;
    *y_it = pt.y;
    *z_it = pt.z;
    *intensity_it = pt.intensity;

    // increment the index to keep track of the pointcloud's size
    ++point_cloud_idx;
    ret = true;
  }
  return ret;
}

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware
