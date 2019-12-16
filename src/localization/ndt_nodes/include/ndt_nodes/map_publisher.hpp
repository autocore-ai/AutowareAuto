// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#ifndef NDT_NODES__MAP_PUBLISHER_HPP_
#define NDT_NODES__MAP_PUBLISHER_HPP_

#include <ndt_nodes/visibility_control.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <ndt/ndt_map.hpp>
#include <string>
#include <memory>

namespace autoware
{
namespace localization
{
namespace ndt_nodes
{

/// Read the pcd file into a PointCloud2 message. Throws if the file cannot be read.
/// \param[in] file_name Name of the pcd file.
/// \param[out] msg PointCloud2 message
void NDT_NODES_PUBLIC read_from_pcd(
  const std::string & file_name,
  sensor_msgs::msg::PointCloud2 & msg);

/// Node to read pcd files, transform to ndt maps and publish the resulting maps in PointCloud2
/// format
class NDT_NODES_PUBLIC NDTMapPublisherNode : public rclcpp::Node
{
public:
  using MapConfig = perception::filters::voxel_grid::Config;
  /// Constructor.
  /// \param node_name Name of node.
  /// \param node_namespace Namespace of node.
  /// \param map_topic The topic the maps will be published to.
  /// \param map_frame The frame of the map.
  /// \param map_config The VoxelGrid configuration of the underlying DynamicNDTMap
  /// \param file_name The name of the file.
  /// \param num_expected_subs Expected number of subscribers to the published maps.
  /// \param init_timeout The time budget the map publisher can use to wait for matching
  /// its recipients. If the publisher cannot match enough recipients,
  /// then it will throw an exception.
  NDTMapPublisherNode(
    const std::string & node_name,
    const std::string & node_namespace,
    const std::string & map_topic,
    const std::string & map_frame,
    const MapConfig & map_config,
    const std::string & file_name,
    const uint32_t num_expected_subs,
    std::chrono::milliseconds init_timeout
  );

  /// Construct with a config file.
  /// \param node_name Name of node.
  /// \param node_namespace Namespace of node.
  NDTMapPublisherNode(
    const std::string & node_name,
    const std::string & node_namespace
  );

  /// Run the publisher. Following actions are executed in order:
  /// 1. Wait until the map publisher matches the configured number of subscribers. If the
  /// configured timeout occurs, it throws an exception.
  /// 2. Load the PCD file into a PointCloud2 message.
  /// 3. Apply the normal distribution transform loaded PointCloud2 message.
  /// 4. Convert the resulting map representation into a `PointCloud2` message and publish.
  void run();

private:
  /// Initialzie and allocate memory for the point clouds that are used as intermediate
  /// representations during  conversions.
  /// \param map_frame Frame of the
  /// \param map_capacity
  void init(const MapConfig & map_config, const std::string & map_frame);

  /// Read the pcd file with filename into a PointCloud2 message, transform it into an NDT
  /// representation and then serialize the ndt representation back into a PointCloud2 message
  /// that can be published.
  /// \param fn File name of the pcd file.
  void load_pcd_file();

  /// Publish the loaded map file. If no new map is loaded, it will publish the
  /// previous map, or an empty map.
  void publish();

  /// Reset the internal point clouds and the ndt map.
  void reset();

  /// Wait until the publisher has discovered enough subscriptions. Throws on timeout.
  /// \param num_expected_subs Number of subscriptions listening to the publisher's topic.
  /// \param match_timeout Time within the discovery is expected to occur.
  void wait_for_matched(const uint32_t num_expected_subs, std::chrono::milliseconds match_timeout);

  /// Iterate over the map representation and convert it into a PointCloud2 message where each voxel
  /// in the map corresponds to a single point in the PointCloud2 field. See the documentation for
  /// the specs and the format of the point cloud message.
  void map_to_pc();

  /// Convenience function to clear the contents of a pointcloud message.
  /// Can be removed when #102 is merged in.
  void reset_pc_msg(sensor_msgs::msg::PointCloud2 & msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;
  std::unique_ptr<ndt::DynamicNDTMap> m_ndt_map_ptr;
  sensor_msgs::msg::PointCloud2 m_map_pc;
  sensor_msgs::msg::PointCloud2 m_source_pc;
  const std::string m_file_name;
  const uint32_t m_num_subs;
  const std::chrono::milliseconds m_timeout_ms;
};

}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware

#endif  // NDT_NODES__MAP_PUBLISHER_HPP_
