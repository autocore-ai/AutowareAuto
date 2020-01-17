// Copyright 2020 Apex.AI, Inc.
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
#include <gtest/gtest.h>

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using autoware::common::types::float32_t;

class RayGroundPclValidationTester : public rclcpp::Node
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

public:
  RayGroundPclValidationTester()
  : Node{"pcl_listener"},
    m_sub_nonground_points{create_subscription<PointCloud2>("nonground_cloud", rclcpp::QoS{50},
        [this](const PointCloud2::SharedPtr msg) {
          m_nonground_points.emplace_back(*msg);
        })},
    m_sub_ground_points{create_subscription<PointCloud2>("ground_cloud", rclcpp::QoS{50},
      [this](const PointCloud2::SharedPtr msg) {
        m_ground_points.emplace_back(*msg);
      })},
  m_pub_raw_points{create_publisher<PointCloud2>("raw_cloud", rclcpp::QoS{50})}
  {
  }

  bool8_t receive_correct_ground_pcls(uint32_t expected_gounrd_pcl_size, uint32_t expected_num)
  {
    bool8_t ret = true;
    if (m_ground_points.size() != expected_num) {
      std::cout << "expected num of pcl not matched" << std::endl;
      std::cout << "actual num = " << m_ground_points.size() << std::endl;
      return false;
    }
    for (std::size_t i = 0; i < m_ground_points.size(); i++) {
      ret = (m_ground_points[i].data.size() == expected_gounrd_pcl_size) && ret;
      std::cout << "ground pc actual size = " << m_ground_points[i].data.size() << std::endl;
    }
    return ret;
  }

  bool8_t receive_correct_nonground_pcls(uint32_t expected_nongnd_pcl_size, uint32_t expected_num)
  {
    bool8_t ret = true;
    if (m_nonground_points.size() != expected_num) {
      std::cout << "expected num of pcl not matched" << std::endl;
      std::cout << "actual num = " << m_nonground_points.size() << std::endl;
      return false;
    }
    for (std::size_t i = 0; i < m_nonground_points.size(); i++) {
      ret = (m_nonground_points[i].data.size() == expected_nongnd_pcl_size) && ret;
      std::cout << "nonground pc actual size = " << m_nonground_points[i].data.size() << std::endl;
    }
    return ret;
  }

  std::vector<PointCloud2> m_nonground_points;
  std::vector<PointCloud2> m_ground_points;
  std::shared_ptr<rclcpp::Subscription<PointCloud2>> m_sub_nonground_points;
  std::shared_ptr<rclcpp::Subscription<PointCloud2>> m_sub_ground_points;
  std::shared_ptr<rclcpp::Publisher<PointCloud2>> m_pub_raw_points;
};

class ray_ground_classifier_pcl_validation : public ::testing::Test
{
protected:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  PointCloud2 create_custom_pcl(
    const std::vector<std::string> field_names,
    const uint32_t cloud_size)
  {
    PointCloud2 msg;
    const auto field_size = field_names.size();
    msg.height = 1U;
    msg.width = cloud_size;
    msg.fields.resize(field_size);
    for (uint32_t i = 0U; i < field_size; i++) {
      msg.fields[i].name = field_names[i];
    }
    msg.point_step = 0U;
    for (uint32_t idx = 0U; idx < field_size; ++idx) {
      msg.fields[idx].offset = static_cast<uint32_t>(idx * sizeof(float32_t));
      msg.fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[idx].count = 1U;
      msg.point_step += static_cast<uint32_t>(sizeof(float32_t));
    }
    const std::size_t capacity = msg.point_step * cloud_size;
    msg.data.clear();
    msg.data.reserve(capacity);
    for (std::size_t i = 0; i < capacity; ++i) {
      msg.data.emplace_back(0U);  // initialize all values equal to 0
    }
    msg.row_step = msg.point_step * msg.width;
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.header.frame_id = "base_link";
    return msg;
  }
  using RayGroundClassifierCloudNode = autoware::perception::filters::ray_ground_classifier_nodes
    ::RayGroundClassifierCloudNode;
  std::shared_ptr<RayGroundClassifierCloudNode> m_ray_gnd_ptr;
  std::shared_ptr<RayGroundPclValidationTester> m_ray_gnd_validation_tester;
  rclcpp::executors::SingleThreadedExecutor m_exec;
  const std::string m_raw_pcl_topic{"raw_cloud"};
  const std::string m_ground_pcl_topic{"ground_cloud"};
  const std::string m_nonground_pcl_topic{"nonground_cloud"};
  const uint32_t m_mini_cloud_size = 10U;
  std::chrono::microseconds m_period = std::chrono::milliseconds(200);
};
