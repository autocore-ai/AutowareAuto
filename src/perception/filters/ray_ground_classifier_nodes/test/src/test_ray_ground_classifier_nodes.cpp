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

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"

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
  PointCloud2::SharedPtr create_custom_pcl(
    const std::vector<std::string> field_names,
    const uint32_t cloud_size)
  {
    PointCloud2::SharedPtr msg = std::make_shared<PointCloud2>();
    const auto field_size = field_names.size();
    msg->height = 1U;
    msg->width = cloud_size;
    msg->fields.resize(field_size);
    for (uint32_t i = 0U; i < field_size; i++) {
      msg->fields[i].name = field_names[i];
    }
    msg->point_step = 0U;
    for (uint32_t idx = 0U; idx < field_size; ++idx) {
      msg->fields[idx].offset = static_cast<uint32_t>(idx * sizeof(float32_t));
      msg->fields[idx].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg->fields[idx].count = 1U;
      msg->point_step += static_cast<uint32_t>(sizeof(float32_t));
    }
    const std::size_t capacity = msg->point_step * cloud_size;
    msg->data.clear();
    msg->data.reserve(capacity);
    for (std::size_t i = 0; i < capacity; ++i) {
      msg->data.emplace_back(0U);  // initialize all values equal to 0
    }
    msg->row_step = msg->point_step * msg->width;
    msg->is_bigendian = false;
    msg->is_dense = false;
    msg->header.frame_id = "base_link";
    return msg;
  }
};

TEST_F(ray_ground_classifier_pcl_validation, has_intensity_and_throw_if_no_xyz_test)
{
  const uint32_t mini_cloud_size = 10U;

  using autoware::perception::filters::ray_ground_classifier_nodes::
  has_intensity_and_throw_if_no_xyz;

  std::vector<std::string> right_field_names{"x", "y", "z", "intensity"};
  std::vector<std::string> not_intensity_field_names{"x", "y", "z", "not_intensity"};
  std::vector<std::string> three_field_names{"x", "y", "z"};
  std::vector<std::string> five_field_names{"x", "y", "z", "intensity", "timestamp"};
  std::vector<std::string> invalid_field_names{"x", "y"};
  std::vector<std::string> wrong_x_field_names{"h", "y", "z"};
  std::vector<std::string> wrong_y_field_names{"x", "h", "z"};
  std::vector<std::string> wrong_z_field_names{"x", "y", "h"};
  const auto correct_pc = create_custom_pcl(right_field_names, mini_cloud_size);
  const auto not_intensity_pc = create_custom_pcl(not_intensity_field_names, mini_cloud_size);
  const auto three_fields_pc = create_custom_pcl(three_field_names, mini_cloud_size);
  const auto five_fields_pc = create_custom_pcl(five_field_names, mini_cloud_size);
  const auto invalid_pc = create_custom_pcl(invalid_field_names, mini_cloud_size);
  const auto no_x_pc = create_custom_pcl(wrong_x_field_names, mini_cloud_size);
  const auto no_y_pc = create_custom_pcl(wrong_y_field_names, mini_cloud_size);
  const auto no_z_pc = create_custom_pcl(wrong_z_field_names, mini_cloud_size);

  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(invalid_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_x_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_y_pc), std::runtime_error);
  EXPECT_THROW(has_intensity_and_throw_if_no_xyz(no_z_pc), std::runtime_error);
  EXPECT_FALSE(has_intensity_and_throw_if_no_xyz(not_intensity_pc));
  EXPECT_FALSE(has_intensity_and_throw_if_no_xyz(three_fields_pc));
  EXPECT_TRUE(has_intensity_and_throw_if_no_xyz(correct_pc));
  EXPECT_TRUE(has_intensity_and_throw_if_no_xyz(five_fields_pc));
}

TEST_F(ray_ground_classifier_pcl_validation, filter_test)
{
  rclcpp::init(0, nullptr);

  using Config = autoware::perception::filters::ray_ground_classifier::Config;
  const Config ray_config{
    0.0,
    20.0,
    7.0,
    70.0,
    0.05,
    3.3,
    3.6,
    5.0,
    -2.5,
    3.5
  };
  using RayAggregator = autoware::perception::filters::ray_ground_classifier::RayAggregator;
  const RayAggregator::Config ray_agg_config{
    -3.14159,
    3.14159,
    0.01,
    512
  };

  using autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode;
  std::shared_ptr<RayGroundClassifierCloudNode> ray_gnd_ptr;
  std::shared_ptr<RayGroundPclValidationTester> ray_gnd_validation_tester;
  const std::string raw_pcl_topic{"raw_cloud"};
  const std::string ground_pcl_topic{"ground_cloud"};
  const std::string nonground_pcl_topic{"nonground_cloud"};
  const uint32_t mini_cloud_size = 10U;

  const uint32_t cloud_size{55000U};
  const char8_t * const frame_id{"base_link"};

  ray_gnd_ptr = std::make_shared<RayGroundClassifierCloudNode>(
    "ray_ground_classifier_cloud_node",
    raw_pcl_topic,
    ground_pcl_topic,
    nonground_pcl_topic,
    frame_id,
    std::chrono::milliseconds(110),
    cloud_size,
    ray_config,
    ray_agg_config
  );
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != ray_gnd_ptr->configure().id()) {
    throw std::runtime_error("Could not configure RayGroundClassifierCloudNode!");
  }
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != ray_gnd_ptr->activate().id()) {
    throw std::runtime_error("Could not activate RayGroundClassifierCloudNode!");
  }

  ray_gnd_validation_tester = std::make_shared<RayGroundPclValidationTester>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(ray_gnd_ptr->get_node_base_interface());
  exec.add_node(ray_gnd_validation_tester);

  std::vector<std::string> five_field_names{"x", "y", "z", "intensity", "timestamp"};
  std::vector<std::string> three_field_names{"x", "y", "z"};
  const auto three_fields_pc = create_custom_pcl(three_field_names, mini_cloud_size);
  const auto five_fields_pc = create_custom_pcl(five_field_names, mini_cloud_size);

  // expected size = 4 bytes * 4 fields * cloud_size
  uint32_t expected_gnd_pcl_size = 4U * 4U * mini_cloud_size;
  uint32_t expected_nongnd_pcl_size = 0U;  // no points will be classified as nonground
  uint32_t expected_num_of_pcl = 2U;

  std::this_thread::sleep_for(std::chrono::milliseconds(500LL));

  ray_gnd_validation_tester->m_pub_raw_points->publish(five_fields_pc);
  // wait for ray_gnd_filter to process 1st pc and publish data
  std::this_thread::sleep_for(std::chrono::milliseconds(500LL));
  exec.spin_some();  // for tester to collect data

  ray_gnd_validation_tester->m_pub_raw_points->publish(three_fields_pc);
  // wait for ray_gnd_filter to process 2nd pc and publish data
  std::this_thread::sleep_for(std::chrono::milliseconds(500LL));
  exec.spin_some();  // for tester to collect data

  // Check all published nonground / ground pointclouds have the expected sizes
  EXPECT_TRUE(ray_gnd_validation_tester->receive_correct_ground_pcls(
      expected_gnd_pcl_size, expected_num_of_pcl));
  EXPECT_TRUE(ray_gnd_validation_tester->receive_correct_nonground_pcls(
      expected_nongnd_pcl_size, expected_num_of_pcl));
}
