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
#include "udp_driver/udp_driver_node.hpp"
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
class VELODYNE_NODES_PUBLIC VelodyneCloudNode
  : public udp_driver::UdpDriverNode<
    typename velodyne_driver::VelodyneTranslator<SensorData>::Packet,
    sensor_msgs::msg::PointCloud2>
{
public:
  using VelodyneTranslatorT = velodyne_driver::VelodyneTranslator<SensorData>;
  using Config = typename VelodyneTranslatorT::Config;
  using Packet = typename VelodyneTranslatorT::Packet;
  using UdpDriverNode = udp_driver::UdpDriverNode<Packet, sensor_msgs::msg::PointCloud2>;

  /// \brief Default constructor, starts driver
  /// \param[in] node_name name of the node for rclcpp internals
  /// \param[in] ip Expected IP of UDP packets
  /// \param[in] port Port that this driver listens to (i.e. sensor device at ip writes to port)
  /// \param[in] frame_id Frame id for the published point cloud messages
  /// \param[in] cloud_size Preallocated capacity (in number of points) for the point cloud messages
  ///                       must be greater than PointBlock::CAPACITY
  /// \param[in] config Config struct with rpm params
  /// \throw std::runtime_error If cloud_size is not sufficiently large
  VelodyneCloudNode(
    const std::string & node_name,
    const std::string & ip,
    const uint16_t port,
    const std::string & frame_id,
    const std::size_t cloud_size,
    const Config & config);

  /// \brief Parameter file constructor
  /// \param[in] node_name Name of this node
  /// \param[in] node_namespace Namespace for this node
  VelodyneCloudNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

protected:
  void init_output(sensor_msgs::msg::PointCloud2 & output) override;
  bool8_t convert(
    const Packet & pkt,
    sensor_msgs::msg::PointCloud2 & output) override;
  bool8_t get_output_remainder(sensor_msgs::msg::PointCloud2 & output) override;

private:
  VelodyneTranslatorT m_translator;
  std::vector<autoware::common::types::PointXYZIF> m_point_block;

  // These next two variables are a minor hack to maintain stateful information across convert()
  // calls. Specifically, it signals to reset any stateful information on the data vector at the top
  // of the convert function
  bool8_t m_published_cloud;
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
