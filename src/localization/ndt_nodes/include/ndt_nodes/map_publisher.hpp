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
#include <tf2_ros/static_transform_broadcaster.h>
#include <voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp>
#include <string>
#include <memory>
#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace localization
{
namespace ndt_nodes
{

/// struct to hold geocentric pose of map origin
/// decribed as a latitude, longitude, and elevation
/// along with an orientation: roll, pitch and yaw
struct geocentric_pose_t
{
  float64_t latitude;
  float64_t longitude;
  float64_t elevation;
  float64_t roll;
  float64_t pitch;
  float64_t yaw;
};

/// Read the map info from a yaml file. Throws if the file cannot be read.
/// \param[in] yaml_file_name Name of the ymal file.
/// \param[out] geo_pose Geocentric pose describing map orgin
void NDT_NODES_PUBLIC read_from_yaml(
  const std::string & yaml_file_name,
  geocentric_pose_t * geo_pose);

/// Read the pcd file into a PointCloud2 message. Throws if the file cannot be read.
/// \param[in] file_name Name of the pcd file.
/// \param[out] msg Pointer to PointCloud2 message
void NDT_NODES_PUBLIC read_from_pcd(
  const std::string & file_name,
  sensor_msgs::msg::PointCloud2 * msg);

/// Node to read pcd files, transform to ndt maps and publish the resulting maps in PointCloud2
/// format
class NDT_NODES_PUBLIC NDTMapPublisherNode : public rclcpp::Node
{
public:
  using MapConfig = perception::filters::voxel_grid::Config;
  using VoxelGrid = perception::filters::voxel_grid_nodes::algorithm::VoxelCloudCentroid;
  /// Constructor.
  /// \param node_name Name of node.
  /// \param node_namespace Namespace of node.
  /// \param map_topic The topic the maps will be published to.
  /// \param map_frame The frame of the map.
  /// \param map_config The VoxelGrid configuration of the underlying DynamicNDTMap
  /// \param pcl_file_name The name of the pcl map data file in pcd format.
  /// \param yaml_file_name The name of the yaml file with map orgin data.
  /// \param viz_map Boolean flag to choose whether to publish a visualizable point cloud.
  /// \param viz_map_topic The topic hte vizualizable map will be published to.
  NDTMapPublisherNode(
    const std::string & node_name,
    const std::string & node_namespace,
    const std::string & map_topic,
    const std::string & map_frame,
    const MapConfig & map_config,
    const std::string & pcl_file_name,
    const std::string & yaml_file_name,
    const bool8_t viz_map = false,
    const std::string & viz_map_topic = ""
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
  /// Initialize and allocate memory for the point clouds that are used as intermediate
  /// representations during  conversions. & setup visualization publisher if required
  /// \param map_frame Frame of the map
  /// \param map_topic Topic name for ndt map
  /// \param viz_map_topic Topic name for map visualization
  void init(
    const std::string & map_frame,
    const std::string & map_topic,
    const std::string & viz_map_topic);

  /// Read the pcd file with filename into a PointCloud2 message, transform it into an NDT
  /// representation and then serialize the ndt representation back into a PointCloud2 message
  /// that can be published.
  /// \param fn File name of the pcd file.
  void load_map();

  void publish_earth_to_map_transform(
    float64_t x, float64_t y, float64_t z,
    float64_t roll, float64_t pitch, float64_t yaw);

  /// Publish the loaded map file. If no new map is loaded, it will publish the
  /// previous map, or an empty map.
  void publish();

  /// Reset the internal point clouds and the ndt map.
  void reset();

  /// Iterate over the map representation and convert it into a PointCloud2 message where each voxel
  /// in the map corresponds to a single point in the PointCloud2 field. See the documentation for
  /// the specs and the format of the point cloud message.
  void map_to_pc();

  /// Use a Voxel Grid filter to downsample the loaded map prior to publishing.
  void downsample_pc();

  /// Convenience function to clear the contents of a pointcloud message.
  /// Can be removed when #102 is merged in.
  void reset_pc_msg(sensor_msgs::msg::PointCloud2 & msg);

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_earth_map_broadcaster;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;
  std::unique_ptr<ndt::DynamicNDTMap> m_ndt_map_ptr;
  sensor_msgs::msg::PointCloud2 m_map_pc;
  sensor_msgs::msg::PointCloud2 m_source_pc;
  sensor_msgs::msg::PointCloud2 m_downsampled_pc;
  const std::string m_pcl_file_name;
  const std::string m_yaml_file_name;
  const bool8_t m_viz_map;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_viz_pub;
  std::unique_ptr<MapConfig> m_map_config_ptr;
  std::unique_ptr<MapConfig> m_viz_map_config_ptr;
  std::unique_ptr<VoxelGrid> m_voxelgrid_ptr;
  // Workaround. TODO(yunus.caliskan): Remove in #380
  rclcpp::TimerBase::SharedPtr m_visualization_timer{nullptr};
};

}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware

#endif  // NDT_NODES__MAP_PUBLISHER_HPP_
