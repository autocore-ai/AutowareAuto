// Copyright 2018 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.


/// \copyright Copyright 2017-2018 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file defines a simple ROS 2 velodyne driver that publishes full point clouds

#ifndef VELODYNE_NODES__VELODYNE_CLOUD_NODE_HPP_
#define VELODYNE_NODES__VELODYNE_CLOUD_NODE_HPP_

#include <string>
#include <vector>
#include "common/types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "udp_driver/udp_driver.hpp"
#include "velodyne_driver/velodyne_translator.hpp"
#include "velodyne_nodes/visibility_control.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using autoware::common::types::bool8_t;

namespace autoware
{
namespace drivers
{
/// \brief Resources for nodes that use the `velodyne_driver`
namespace velodyne_nodes
{

/// Template class for the velodyne driver node that receives veldyne `packet`s via
/// UDP, converts the packet into a PointCloud2 message and publishes this cloud.
/// \tparam SensorData SensorData implementation for the specific velodyne sensor model.
template<typename SensorData>
class VELODYNE_NODES_PUBLIC VelodyneCloudNode : public rclcpp::Node
{
public:
  using VelodyneTranslatorT = velodyne_driver::VelodyneTranslator<SensorData>;
  using Config = typename VelodyneTranslatorT::Config;
  using Packet = typename VelodyneTranslatorT::Packet;

  VelodyneCloudNode(const std::string & node_name, const rclcpp::NodeOptions & options);

  /// Handle data packet from the udp driver
  /// \param buffer Data from the udp driver
  void receiver_callback(const std::vector<uint8_t> & buffer);

protected:
  void init_output(sensor_msgs::msg::PointCloud2 & output);
  bool8_t convert(
    const Packet & pkt,
    sensor_msgs::msg::PointCloud2 & output);
  bool8_t get_output_remainder(sensor_msgs::msg::PointCloud2 & output);

private:
  void init_udp_driver();

  IoContext m_io_cxt;
  ::drivers::udp_driver::UdpDriver m_udp_driver;
  VelodyneTranslatorT m_translator;
  std::vector<autoware::common::types::PointXYZIF> m_point_block;

  std::string m_ip;
  uint16_t m_port;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pc2_pub_ptr;
  sensor_msgs::msg::PointCloud2 m_pc2_msg{};
  bool m_published_cloud = false;
  // Keeps track of where you left off on the converted point block in case you needed to publish
  // a point cloud in the middle of processing it
  uint32_t m_remainder_start_idx;
  // keeps track of the constructed point cloud to continue growing it with new data
  uint32_t m_point_cloud_idx;
  autoware::common::lidar_utils::PointCloudIts m_point_cloud_its;
  const std::string m_frame_id;
  const std::size_t m_cloud_size;
};  // class VelodyneCloudNode

using VLP16DriverNode = VelodyneCloudNode<velodyne_driver::VLP16Data>;
using VLP32CDriverNode = VelodyneCloudNode<velodyne_driver::VLP32CData>;
using VLS128DriverNode = VelodyneCloudNode<velodyne_driver::VLS128Data>;
}  // namespace velodyne_nodes
}  // namespace drivers
}  // namespace autoware

#endif  // VELODYNE_NODES__VELODYNE_CLOUD_NODE_HPP_
