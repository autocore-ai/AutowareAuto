// Copyright 2020 the Autoware Foundation
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

#include <GeographicLib/Geocentric.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <ndt/ndt_map_publisher.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <yaml-cpp/yaml.h>

#include <string>
#include <utility>

namespace autoware
{
namespace localization
{
namespace ndt
{

void read_from_yaml(
  const std::string & yaml_file_name,
  geodetic_pose_t * geo_pose)
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
  // TODO(yunus.caliskan): Consider replacing the logic here with pointcloud_msg_wrapper once
  // It supports the padding in the structs with heterogeneous fields.
  pcl::PCLPointCloud2 pcl_cloud;
  if (pcl::io::loadPCDFile(file_name, pcl_cloud) == -1) {  // load the file
    throw std::runtime_error(std::string("PCD file ") + file_name + " could not be loaded.");
  }
  if (pcl_cloud.data.size() == 0) {
    throw std::runtime_error("PCD cloud empty\n");
  }

  // Convert to sensor_msgs in order to check the available fields
  sensor_msgs::msg::PointCloud2 cloud;
  pcl_conversions::moveFromPCL(pcl_cloud, cloud);

  // Ensure that we have at least the x, y, z fields and check whether we have intensity
  const auto has_intensity = common::lidar_utils::has_intensity_and_throw_if_no_xyz(cloud);
  if (has_intensity && msg->fields.size() == 4U &&
    msg->fields[3U].datatype == sensor_msgs::msg::PointField::FLOAT32)
  {
    // Quick path: the data already has the desired format
    *msg = std::move(cloud);
    return;
  }

  // We don't have intensity of the correct format
  // Set up a new point cloud with the correct fields
  sensor_msgs::msg::PointCloud2 adjusted_cloud;
  const size_t num_points = cloud.data.size() / cloud.point_step;

  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> modifier{
    adjusted_cloud, msg->header.frame_id};
  modifier.reserve(num_points);

  // Copy x, y, z into it
  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t * src = cloud.data.data() + cloud.point_step * i;
    uint8_t * dest = adjusted_cloud.data.data() + adjusted_cloud.point_step * i;
    std::memcpy(dest, src, 3 * sizeof(float32_t));
    // If intensity exists, copy it into the new cloud, otherwise, set it to 0.0
    // This would be faster with separate loops, but this function isn't a hotspot
    float intensity = 0.0f;
    if (has_intensity) {
      const size_t intensity_offset = cloud.point_step * i + 3 * sizeof(float32_t);
      if (msg->fields[3U].datatype == sensor_msgs::msg::PointField::FLOAT32) {
        std::memcpy(&intensity, cloud.data.data() + intensity_offset, sizeof(float32_t));
      } else if (msg->fields[3U].datatype == sensor_msgs::msg::PointField::UINT8) {
        intensity = static_cast<float32_t>(cloud.data[intensity_offset]);
      } else {
        throw std::runtime_error("intensity datatype is not float or uint8_t");
      }
    }
    uint8_t * dest_intensity = adjusted_cloud.data.data() + adjusted_cloud.point_step * i + 3 *
      sizeof(float32_t);
    std::memcpy(dest_intensity, &intensity, sizeof(float32_t));
  }

  *msg = std::move(adjusted_cloud);
}

geocentric_pose_t load_map(
  const std::string & yaml_file_name,
  const std::string & pcl_file_name,
  sensor_msgs::msg::PointCloud2 & pc_out)
{
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>{pc_out}.clear();
  geodetic_pose_t geodetic_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  if (!yaml_file_name.empty()) {
    read_from_yaml(yaml_file_name, &geodetic_pose);
  } else {
    throw std::runtime_error("YAML file name empty\n");
  }

  if (!pcl_file_name.empty()) {
    read_from_pcd(pcl_file_name, &pc_out);
  } else {
    throw std::runtime_error("PCD file name empty\n");
  }

  float64_t x(0.0), y(0.0), z(0.0);

  GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());

  earth.Forward(
    geodetic_pose.latitude,
    geodetic_pose.longitude,
    geodetic_pose.elevation,
    x, y, z);

  return {x, y, z, geodetic_pose.roll, geodetic_pose.pitch, geodetic_pose.yaw};
}
}  // namespace ndt
}  // namespace localization
}  // namespace autoware
