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

#ifndef ROSBAG2_MORIYAMA_CONVERTER__DATASET_CONVERTER_HPP_
#define ROSBAG2_MORIYAMA_CONVERTER__DATASET_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nmea_msgs/msg/sentence.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;

namespace rosbag2_moriyama_converter
{
class DatasetConverterNode : public rclcpp::Node
{
public:
  explicit DatasetConverterNode(const rclcpp::NodeOptions & options);

private:
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;

  void nmea_callback(const nmea_msgs::msg::Sentence::SharedPtr msg) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

  rclcpp::Subscription<nmea_msgs::msg::Sentence>::SharedPtr nmea_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_publisher_;
};

}  // namespace rosbag2_moriyama_converter

#endif  // ROSBAG2_MORIYAMA_CONVERTER__DATASET_CONVERTER_HPP_
