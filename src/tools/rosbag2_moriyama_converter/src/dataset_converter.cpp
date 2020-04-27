// Copyright 2020 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rosbag2_moriyama_converter/dataset_converter.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cctype>
#include <memory>
#include <regex>
#include <string>
#include <vector>

using std::placeholders::_1;

rosbag2_moriyama_converter::DatasetConverterNode::DatasetConverterNode(
  const rclcpp::NodeOptions & options)
: Node("dataset_converter", options)
{
  point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/points_raw", 10, std::bind(&DatasetConverterNode::point_cloud_callback, this, _1));
  point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar_front/points_raw", 10);

  nmea_subscription_ = this->create_subscription<nmea_msgs::msg::Sentence>(
    "/nmea_sentence", 10, std::bind(&DatasetConverterNode::nmea_callback, this, _1));
  nav_sat_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gnss/fix",
      10);
}

void
rosbag2_moriyama_converter::DatasetConverterNode::point_cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
{
  msg->header.frame_id = "lidar_front";
  point_cloud_publisher_->publish(*msg);
}

void
rosbag2_moriyama_converter::DatasetConverterNode::nmea_callback(
  const nmea_msgs::msg::Sentence::SharedPtr msg) const
{
  // Trim NMEA sequence
  auto ltrim = std::find_if_not(msg->sentence.begin(), msg->sentence.end(), [](int c) {
        return std::isspace(c);
      });
  auto rtrim = std::find_if_not(msg->sentence.rbegin(), msg->sentence.rend(), [](int c) {
        return std::isspace(c);
      }).base();
  auto trimmed_sentence = std::string(ltrim, rtrim);

  std::regex regex{","};
  std::sregex_token_iterator it{std::begin(trimmed_sentence), std::end(trimmed_sentence), regex,
    -1};
  std::vector<std::string> nmea_parts{it, {}};
  if (!nmea_parts.empty()) {
    if ("$GPGGA" == nmea_parts[0] && nmea_parts.size() >= 10) {
      auto latitude = std::atoi(
        nmea_parts[2].substr(0, 2).c_str()) +
        std::atof(nmea_parts[2].substr(2).c_str()) / 60.0;
      if (nmea_parts[3] == "S") {
        latitude = -latitude;
      }
      auto longitude = std::atoi(
        nmea_parts[4].substr(0, 3).c_str()) +
        std::atof(nmea_parts[4].substr(3).c_str()) / 60.0;
      if (nmea_parts[5] == "W") {
        longitude = -longitude;
      }
      auto altitude = std::atof(nmea_parts[9].c_str());
      auto nav_sat_fix = sensor_msgs::msg::NavSatFix();
      nav_sat_fix.header.stamp = msg->header.stamp;
      nav_sat_fix.header.frame_id = "gnss";
      nav_sat_fix.latitude = latitude;
      nav_sat_fix.longitude = longitude;
      nav_sat_fix.altitude = altitude;
      nav_sat_fix_publisher_->publish(nav_sat_fix);
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(rosbag2_moriyama_converter::DatasetConverterNode)
