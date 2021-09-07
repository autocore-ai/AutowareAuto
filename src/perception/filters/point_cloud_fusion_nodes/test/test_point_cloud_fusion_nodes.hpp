// Copyright 2019-2021 the Autoware Foundation
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

#ifndef TEST_POINT_CLOUD_FUSION_NODES_HPP_
#define TEST_POINT_CLOUD_FUSION_NODES_HPP_

#include <gtest/gtest.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <point_cloud_fusion_nodes/point_cloud_fusion_node.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <common/types.hpp>
#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

class TestPCF : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    rclcpp::shutdown();
  }
};

sensor_msgs::msg::PointCloud2 make_pc(
  std::vector<int32_t> seeds,
  builtin_interfaces::msg::Time stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  autoware::common::lidar_utils::init_pcl_msg(msg, "base_link", seeds.size());

  uint32_t pidx = 0;
  for (auto seed : seeds) {
    autoware::common::types::PointXYZIF pt;
    pt.x = seed;
    pt.y = seed;
    pt.z = seed;
    pt.intensity = seed;
    autoware::common::lidar_utils::add_point_to_cloud(msg, pt, pidx);
  }

  msg.header.stamp = stamp;

  return msg;
}

void check_pcl_eq(sensor_msgs::msg::PointCloud2 & msg1, sensor_msgs::msg::PointCloud2 & msg2)
{
  EXPECT_EQ(msg1.width, msg2.width);
  EXPECT_EQ(msg1.header.frame_id, msg2.header.frame_id);
  EXPECT_EQ(msg1.header.stamp, msg2.header.stamp);


  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it_1(msg1, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it_1(msg1, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it_1(msg1, "z");
  sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_1(msg1, "intensity");

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it_2(msg2, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it_2(msg2, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it_2(msg2, "z");
  sensor_msgs::PointCloud2ConstIterator<float32_t> intensity_it_2(msg2, "intensity");

  while (x_it_1 != x_it_1.end() &&
    y_it_1 != y_it_1.end() &&
    z_it_1 != z_it_1.end() &&
    intensity_it_1 != intensity_it_1.end() &&
    x_it_2 != x_it_2.end() &&
    y_it_2 != y_it_2.end() &&
    z_it_2 != z_it_2.end() &&
    intensity_it_2 != intensity_it_2.end()
  )
  {
    EXPECT_FLOAT_EQ(*x_it_1, *x_it_2);
    EXPECT_FLOAT_EQ(*y_it_1, *y_it_2);
    EXPECT_FLOAT_EQ(*z_it_1, *z_it_2);
    EXPECT_FLOAT_EQ(*intensity_it_1, *intensity_it_2);

    ++x_it_1;
    ++y_it_1;
    ++z_it_1;
    ++intensity_it_1;

    ++x_it_2;
    ++y_it_2;
    ++z_it_2;
    ++intensity_it_2;
  }

  // Operator== is not defined for some reason
  EXPECT_FALSE(x_it_1 != x_it_1.end());
  EXPECT_FALSE(y_it_1 != y_it_1.end());
  EXPECT_FALSE(z_it_1 != z_it_1.end());
  EXPECT_FALSE(intensity_it_1 != intensity_it_1.end());

  EXPECT_FALSE(x_it_2 != x_it_2.end());
  EXPECT_FALSE(y_it_2 != y_it_2.end());
  EXPECT_FALSE(z_it_2 != z_it_2.end());
  EXPECT_FALSE(intensity_it_2 != intensity_it_2.end());
}

builtin_interfaces::msg::Time to_msg_time(
  const std::chrono::system_clock::time_point time_point)
{
  const auto tse = time_point.time_since_epoch();
  if (tse < std::chrono::nanoseconds(0LL)) {
    throw std::invalid_argument("ROS 2 builtin interfaces time does not support negative a epoch.");
  }
  builtin_interfaces::msg::Time result;

  result.sec = static_cast<decltype(result.sec)>(
    std::chrono::duration_cast<std::chrono::seconds>(tse).count());

  result.nanosec = static_cast<decltype(result.nanosec)>((
      tse - std::chrono::duration_cast<std::chrono::seconds>(tse)).count());

  return result;
}

TEST_F(TestPCF, TestBasicFusion) {
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("number_of_sources", 2);
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("cloud_size", static_cast<int64_t>(55000U));

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  auto pcf_node =
    std::make_shared<autoware::perception::filters::point_cloud_fusion_nodes::PointCloudFusionNode>(
    node_options);

  bool8_t test_completed = false;
  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);
  auto t1 = to_msg_time(time0 + std::chrono::nanoseconds(1));
  auto a_later_time = to_msg_time(time0 + std::chrono::milliseconds(200));

  auto dummy_pc = make_pc({0, 0, 0}, a_later_time);
  auto pc1 = make_pc({1, 2, 3}, t0);
  auto pc2 = make_pc({4, 5, 6}, t1);
  auto expected_result = make_pc({1, 2, 3, 4, 5, 6}, t1);

  auto pub_ptr1 = pcf_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input_topic1",
    rclcpp::QoS(10));
  auto pub_ptr2 = pcf_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input_topic2",
    rclcpp::QoS(10));

  auto handle_concat =
    [&expected_result, &test_completed](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    -> void {
      check_pcl_eq(*msg, expected_result);
      test_completed = true;
    };

  auto sub_ptr = pcf_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "output_topic",
    rclcpp::QoS(10), handle_concat);

  pub_ptr1->publish(pc1);
  pub_ptr2->publish(pc2);

  // publishing for a second time to give a reference point for fusion.
  pub_ptr1->publish(dummy_pc);

  auto start_time = std::chrono::system_clock::now();
  auto max_test_dur = std::chrono::seconds(1);
  auto timed_out = false;

  while (rclcpp::ok() && !test_completed) {
    rclcpp::spin_some(pcf_node);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    if (std::chrono::system_clock::now() - start_time > max_test_dur) {
      timed_out = true;
      break;
    }
  }
  EXPECT_FALSE(timed_out);
  EXPECT_TRUE(test_completed);
}

#endif  // TEST_POINT_CLOUD_FUSION_NODES_HPP_
