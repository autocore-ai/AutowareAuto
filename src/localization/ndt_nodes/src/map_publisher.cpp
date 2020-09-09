// Copyright 2019 Apex.AI, Inc.
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

#include <common/types.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <ndt_nodes/map_publisher.hpp>
#include <pcl/io/pcd_io.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace localization
{
namespace ndt_nodes
{

void read_from_yaml(
  const std::string & yaml_file_name,
  geocentric_pose_t * geo_pose)
{
  try {
    YAML::Node map_info = YAML::LoadFile(yaml_file_name);
    if (map_info["map_config"]) {
      if (map_info["map_config"]["latitude"] &&
        map_info["map_config"]["longitude"] &&
        map_info["map_config"]["elevation"])
      {
        geo_pose->latitude = map_info["map_config"]["latitude"].as<double>();
        geo_pose->longitude = map_info["map_config"]["longitude"].as<double>();
        geo_pose->elevation = map_info["map_config"]["elevation"].as<double>();
      } else {
        throw std::runtime_error("Yaml file: map origin not found\n");
      }
      if (map_info["map_config"]["roll"]) {
        geo_pose->roll = map_info["map_config"]["roll"].as<double>();
      }
      if (map_info["map_config"]["pitch"]) {
        geo_pose->pitch = map_info["map_config"]["pitch"].as<double>();
      }
      if (map_info["map_config"]["yaw"]) {
        geo_pose->yaw = map_info["map_config"]["yaw"].as<double>();
      }
    } else {
      throw std::runtime_error("Yaml file: map config not found\n");
    }
  } catch (const YAML::BadFile & ex) {
    throw std::runtime_error("Yaml file not found\n");
  } catch (const YAML::ParserException & ex) {
    throw std::runtime_error("Yaml syntax error\n");
  }
}

void read_from_pcd(const std::string & file_name, sensor_msgs::msg::PointCloud2 * msg)
{
  pcl::PointCloud<pcl::PointXYZI> cloud{};
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, cloud) == -1) {  // load the file
    throw std::runtime_error("PCD file not found.");
  }
  if (!(cloud.size() > 0)) {
    throw std::runtime_error("PCD Cloud empty\n");
  }
  // TODO(yunus.caliskan): validate the map format. #102
  common::lidar_utils::resize_pcl_msg(*msg, cloud.size());

  auto id = 0U;
  for (const auto & pt : cloud) {
    common::types::PointXYZIF point{pt.x, pt.y, pt.z, pt.intensity,
      static_cast<uint16_t>(id)};
    common::lidar_utils::add_point_to_cloud(*msg, point, id);    // id is incremented inside
  }
}

NDTMapPublisherNode::NDTMapPublisherNode(
  const rclcpp::NodeOptions & node_options
)
: Node("ndt_map_publisher_node", node_options),
  m_pcl_file_name(declare_parameter("map_pcd_file").get<std::string>()),
  m_yaml_file_name(declare_parameter("map_yaml_file").get<std::string>()),
  m_viz_map(declare_parameter("viz_map", false))
{
  using PointXYZ = perception::filters::voxel_grid::PointXYZ;
  PointXYZ min_point;
  min_point.x =
    static_cast<float32_t>(declare_parameter("map_config.min_point.x").get<float32_t>());
  min_point.y =
    static_cast<float32_t>(declare_parameter("map_config.min_point.y").get<float32_t>());
  min_point.z =
    static_cast<float32_t>(declare_parameter("map_config.min_point.z").get<float32_t>());
  PointXYZ max_point;
  max_point.x =
    static_cast<float32_t>(declare_parameter("map_config.max_point.x").get<float32_t>());
  max_point.y =
    static_cast<float32_t>(declare_parameter("map_config.max_point.y").get<float32_t>());
  max_point.z =
    static_cast<float32_t>(declare_parameter("map_config.max_point.z").get<float32_t>());
  PointXYZ voxel_size;
  voxel_size.x =
    static_cast<float32_t>(declare_parameter("map_config.voxel_size.x").get<float32_t>());
  voxel_size.y =
    static_cast<float32_t>(declare_parameter("map_config.voxel_size.y").get<float32_t>());
  voxel_size.z =
    static_cast<float32_t>(declare_parameter("map_config.voxel_size.z").get<float32_t>());
  const std::size_t capacity =
    static_cast<std::size_t>(declare_parameter("map_config.capacity").get<std::size_t>());
  const std::string map_frame = declare_parameter("map_frame").get<std::string>();
  const std::string map_topic = "ndt_map";
  const std::string viz_map_topic = "viz_ndt_map";

  m_map_config_ptr = std::make_unique<MapConfig>(min_point, max_point, voxel_size, capacity);
  init(map_frame, map_topic, viz_map_topic);
  run();
}

void NDTMapPublisherNode::init(
  const std::string & map_frame,
  const std::string & map_topic,
  const std::string & viz_map_topic)
{
  m_ndt_map_ptr = std::make_unique<ndt::DynamicNDTMap>(*m_map_config_ptr);

  common::lidar_utils::init_pcl_msg(m_source_pc, map_frame);
  common::lidar_utils::init_pcl_msg(m_map_pc, map_frame, m_map_config_ptr->get_capacity(), 10U,
    "x", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "y", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "z", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_xx", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_xy", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_xz", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_yy", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_yz", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_zz", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "cell_id", 2U, sensor_msgs::msg::PointField::UINT32);

  m_pub = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic,
      rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());

  if (m_viz_map) {   // create a publisher for map_visualization
    using PointXYZ = perception::filters::voxel_grid::PointXYZ;

    m_viz_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
      viz_map_topic, rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());

    // TODO(jwhitleywork) Hard-coded voxel size and capacity.
    // To be removed when #380 is in.
    PointXYZ viz_voxel_size;
    viz_voxel_size.x = 0.4F;
    viz_voxel_size.y = 0.4F;
    viz_voxel_size.z = 0.4F;
    m_viz_map_config_ptr = std::make_unique<MapConfig>(
      m_map_config_ptr->get_min_point(),
      m_map_config_ptr->get_max_point(),
      viz_voxel_size,
      10000000U);

    // Initialize Voxel Grid and output message for downsampling map
    common::lidar_utils::init_pcl_msg(m_downsampled_pc, map_frame);
    m_voxelgrid_ptr = std::make_unique<VoxelGrid>(*m_viz_map_config_ptr);

    // Periodic publishing is a temp. hack until the rviz in ade has transient_local qos support.
    // TODO(yunus.caliskan): Remove the loop and publish only once after #380
    m_visualization_timer = create_wall_timer(std::chrono::seconds(1),
        [this]() {
          if (m_downsampled_pc.width > 0U) {
            m_viz_pub->publish(m_downsampled_pc);
          }
        });
  }

  m_pub_earth_map = create_publisher<tf2_msgs::msg::TFMessage>("/tf_static",
      rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());
}

void NDTMapPublisherNode::run()
{
  load_map();
  publish();
}

void NDTMapPublisherNode::load_map()
{
  reset_pc_msg(m_map_pc);  // TODO(yunus.caliskan): Change in #102
  reset_pc_msg(m_source_pc);  // TODO(yunus.caliskan): Change in #102

  geocentric_pose_t geo_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  if (!m_yaml_file_name.empty()) {
    read_from_yaml(m_yaml_file_name, &geo_pose);
  } else {
    throw std::runtime_error("YAML file name empty\n");
  }

  if (!m_pcl_file_name.empty()) {
    read_from_pcd(m_pcl_file_name, &m_source_pc);
  } else {
    throw std::runtime_error("PCD file name empty\n");
  }

  float64_t x(0.0), y(0.0), z(0.0);

  GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());

  earth.Forward(
    geo_pose.latitude,
    geo_pose.longitude,
    geo_pose.elevation,
    x, y, z);
  publish_earth_to_map_transform(x, y, z,
    geo_pose.roll, geo_pose.pitch, geo_pose.yaw);

  m_ndt_map_ptr->insert(m_source_pc);
  map_to_pc();

  if (m_viz_map) {
    reset_pc_msg(m_downsampled_pc);
    downsample_pc();
  }
}

void NDTMapPublisherNode::publish_earth_to_map_transform(
  float64_t x, float64_t y, float64_t z,
  float64_t roll, float64_t pitch, float64_t yaw)
{
  geometry_msgs::msg::TransformStamped tf;

  tf.transform.translation.x = x;
  tf.transform.translation.y = y;
  tf.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf.header.stamp = rclcpp::Clock().now();
  tf.transform.rotation = tf2::toMsg(q);
  tf.header.frame_id = "earth";
  tf.child_frame_id = "map";

  tf2_msgs::msg::TFMessage static_tf_msg;
  static_tf_msg.transforms.push_back(tf);

  m_transform_pub_timer = create_wall_timer(std::chrono::seconds(1),
      [this, static_tf_msg]() {
        m_pub_earth_map->publish(static_tf_msg);
      });

  m_pub_earth_map->publish(static_tf_msg);
}

void NDTMapPublisherNode::map_to_pc()
{
  reset_pc_msg(m_map_pc);
  common::lidar_utils::resize_pcl_msg(m_map_pc, m_ndt_map_ptr->size());

  // TODO(yunus.caliskan): Make prettier -> #102
  sensor_msgs::PointCloud2Iterator<ndt::Real> x_it(m_map_pc, "x");
  sensor_msgs::PointCloud2Iterator<ndt::Real> y_it(m_map_pc, "y");
  sensor_msgs::PointCloud2Iterator<ndt::Real> z_it(m_map_pc, "z");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_xx_it(m_map_pc, "icov_xx");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_xy_it(m_map_pc, "icov_xy");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_xz_it(m_map_pc, "icov_xz");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_yy_it(m_map_pc, "icov_yy");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_yz_it(m_map_pc, "icov_yz");
  sensor_msgs::PointCloud2Iterator<ndt::Real> icov_zz_it(m_map_pc, "icov_zz");
  sensor_msgs::PointCloud2Iterator<uint32_t> cell_id_it(m_map_pc, "cell_id");

  auto num_used_cells = 0U;
  for (const auto & vx_it : *m_ndt_map_ptr) {
    if (!  // No `==` operator defined for PointCloud2Iterators
      (y_it != y_it.end() &&
      z_it != z_it.end() &&
      icov_xx_it != icov_xx_it.end() &&
      icov_xy_it != icov_xy_it.end() &&
      icov_xz_it != icov_xz_it.end() &&
      icov_yy_it != icov_yy_it.end() &&
      icov_yz_it != icov_yz_it.end() &&
      icov_zz_it != icov_zz_it.end() &&
      cell_id_it != cell_id_it.end()))
    {
      // This should not occur as the cloud is resized to the map's size.
      throw std::length_error("NDTMapPublisherNode: NDT map is larger than the map point cloud.");
    }
    const auto & vx = vx_it.second;
    if (!vx.usable()) {
      // Voxel doesn't have enough points to be used in NDT
      continue;
    }

    const auto inv_covariance_opt = vx.inverse_covariance();
    if (!inv_covariance_opt) {
      // Voxel covariance is not invertible
      continue;
    }

    const auto & centroid = vx.centroid();
    const auto & inv_covariance = inv_covariance_opt.value();
    *(x_it) = centroid(0U);
    *(y_it) = centroid(1U);
    *(z_it) = centroid(2U);
    *(icov_xx_it) = inv_covariance(0U, 0U);
    *(icov_xy_it) = inv_covariance(0U, 1U);
    *(icov_xz_it) = inv_covariance(0U, 2U);
    *(icov_yy_it) = inv_covariance(1U, 1U);
    *(icov_yz_it) = inv_covariance(1U, 2U);
    *(icov_zz_it) = inv_covariance(2U, 2U);

    // There are cases where the centroid of a voxel does get indexed to another voxel. To prevent
    // ID mismatches while transferring the map. The index from the voxel grid config is used.
    const auto correct_idx = m_map_config_ptr->index(centroid);

    std::memcpy(&cell_id_it[0U], &(correct_idx), sizeof(correct_idx));
    ++x_it;
    ++y_it;
    ++z_it;
    ++icov_xx_it;
    ++icov_xy_it;
    ++icov_xz_it;
    ++icov_yy_it;
    ++icov_yz_it;
    ++icov_zz_it;
    ++cell_id_it;
    ++num_used_cells;
  }
  // Resize to throw out unused cells.
  common::lidar_utils::resize_pcl_msg(m_map_pc, num_used_cells);
}

void NDTMapPublisherNode::downsample_pc()
{
  common::lidar_utils::resize_pcl_msg(m_downsampled_pc, m_source_pc.data.size());

  try {
    m_voxelgrid_ptr->insert(m_source_pc);
    m_downsampled_pc = m_voxelgrid_ptr->get();
  } catch (const std::exception & e) {
    std::string err_msg{e.what()};
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
  } catch (...) {
    std::string err_msg{"Unknown error occurred in "};
    err_msg += get_name();
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    throw;
  }
}

void NDTMapPublisherNode::publish()
{
  if (m_map_pc.width > 0U) {
    m_pub->publish(m_map_pc);
  }
}

void NDTMapPublisherNode::reset()
{
  reset_pc_msg(m_map_pc);
  reset_pc_msg(m_source_pc);

  if (m_viz_map) {
    reset_pc_msg(m_downsampled_pc);
  }

  m_ndt_map_ptr->clear();
}

void NDTMapPublisherNode::reset_pc_msg(sensor_msgs::msg::PointCloud2 & msg)
{
  auto dummy_idx = 0U;  // TODO(yunus.caliskan): Change in #102
  common::lidar_utils::reset_pcl_msg(msg, 0U, dummy_idx);
}

}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::localization::ndt_nodes::NDTMapPublisherNode)
