/// \copyright Copyright 2017-2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This class defines common functions and classes to work with pointclouds

#ifndef LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_
#define LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <lidar_utils/lidar_types.hpp>
#include <lidar_utils/visibility_control.hpp>

#include <string>
#include <atomic>
#include <memory>
#include <vector>

namespace autoware
{
namespace common
{
namespace lidar_utils
{
/// max number of points in a scan for VLP16s, assuming 300 rpm = 5hz: 57870.3703 points per full
/// rotation
static const uint32_t MAX_SCAN_POINTS = 57872U;

/// Point cloud iterator wrapper, that allows to reuse iterators. Reinitializing
/// of iterators is quite costly due to the implementation of the
/// PointCloud2IteratorBase<...>::set_field method.
class LIDAR_UTILS_PUBLIC PointCloudIts
{
public:
  /// \brief Creates new iterator wrapper with space reserved for 4 iterators,
  /// namely x, y, z and intensity
  PointCloudIts();

  /// \brief Resets the iterators for the given cloud to the given index
  /// \param[in] cloud the point cloud to reset the iterators to
  /// \param[in] idx the index/offset to advance the iterators to
  void reset(sensor_msgs::msg::PointCloud2 & cloud, uint32_t idx = 0);

  /// \brief Returns iterator for the "x" field
  inline sensor_msgs::PointCloud2Iterator<float> & x_it()
  {
    return m_its[0];
  }

  /// \brief Returns iterator for the "y" field
  inline sensor_msgs::PointCloud2Iterator<float> & y_it()
  {
    return m_its[1];
  }

  /// \brief Returns iterator for the "z" field
  inline sensor_msgs::PointCloud2Iterator<float> & z_it()
  {
    return m_its[2];
  }

  /// \brief Returns iterator for the "intensity" field
  sensor_msgs::PointCloud2Iterator<float> & intensity_it()
  {
    return m_its[3];
  }

private:
  /// Internal storage of the iterators
  ::std::vector<sensor_msgs::PointCloud2Iterator<float>> m_its;
};

/// \brief initializes header information for point cloud for x, y, z and intensity
/// \param[out] msg a point cloud message to initialize
/// \param[in] frame_id the name of the frame for the point cloud (assumed fixed)
/// \param[in] size number of points to preallocate for underyling data array
LIDAR_UTILS_PUBLIC void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size = static_cast<std::size_t>(MAX_SCAN_POINTS));

LIDAR_UTILS_PUBLIC bool add_point_to_cloud(
  PointCloudIts & cloud_its,
  const autoware::common::lidar_utils::PointXYZIF & pt,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC bool add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::lidar_utils::PointXYZIF & pt,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC void reset_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t size,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC void resize_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t new_size);

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware

#endif  // LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_
