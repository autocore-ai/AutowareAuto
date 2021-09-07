// Copyright 2021 The Autoware Foundation
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

#include <common/types.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <random>
#include <memory>
#include <string>
#include <vector>
#include "gtest/gtest.h"
#include "polygon_remover_nodes/polygon_remover_node.hpp"

using autoware::common::types::PointXYZI;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using autoware::common::types::float32_t;

geometry_msgs::msg::Point32 make_point_geo(float x, float y, float z)
{
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

/// \brief Generates a point cloud with following properties:
/// There are \p count_points_within_rect points within the rectangle defined by bounds:
/// \p bound_x_min,
/// \p bound_x_max,
/// \p bound_y_min,
/// \p bound_y_max
/// There are \p count_points_outside_rect points outside the rectangle
/// with outer bounds(-100F,100F)
/// The z values of the points will be random within (-100F,100F)
/// \return
PointCloud2::SharedPtr generate_cloud_rect_counted(
  const uint32_t count_points_within_rect,
  const uint32_t count_points_outside_rect,
  const float32_t bound_x_min,
  const float32_t bound_x_max,
  const float32_t bound_y_min,
  const float32_t bound_y_max)
{
  PointCloud2::SharedPtr cloud_input_ptr = std::make_shared<PointCloud2>();
  using CloudModifier = point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>;
  CloudModifier cloud_modifier_input(*cloud_input_ptr, "");

  // Populate it with random points within a selected area
  uint64_t seed = 19940426;
  std::mt19937 mt(seed);
  std::uniform_real_distribution<float> dist_within_x(
    bound_x_min + 0.01F, bound_x_max - 0.01F);
  std::uniform_real_distribution<float> dist_within_y(
    bound_y_min + 0.01F, bound_y_max - 0.01F);
  // z shoudln't matter
  std::uniform_real_distribution<float> dist_within_z(
    -100.0F, 100.0F);

  auto generate_random_point_within_rect =
    [&dist_within_x, &dist_within_y, &dist_within_z, &mt]() {
      PointXYZI point_xyzi;
      point_xyzi.x = dist_within_x(mt);
      point_xyzi.y = dist_within_y(mt);
      point_xyzi.z = dist_within_z(mt);
      return point_xyzi;
    };

  std::uniform_real_distribution<float> dist_big_scope(-100.0F, 100.0F);

  auto get_value_outside_bounds =
    [&dist_big_scope, &mt](float bound_low, float bound_high) {
      float value = dist_big_scope(mt);
      // regenerate if within bounds
      while (value >= bound_low + 0.01F && value <= bound_high - 0.01F) {
        value = dist_big_scope(mt);
      }
      return value;
    };

  auto generate_random_point_outside_rect =
    [&get_value_outside_bounds,
      bound_x_min,
      bound_x_max,
      bound_y_min,
      bound_y_max,
      &dist_within_z,
      &mt]() {
      PointXYZI point_xyzi;
      point_xyzi.x = get_value_outside_bounds(bound_x_min, bound_x_max);
      point_xyzi.y = get_value_outside_bounds(bound_y_min, bound_y_max);
      point_xyzi.z = dist_within_z(mt);
      return point_xyzi;
    };

  // generate random points within the rectangle
  cloud_modifier_input.reserve(count_points_within_rect);
  for (uint32_t i = 0; i < count_points_within_rect; ++i) {
    auto point = generate_random_point_within_rect();
    std::cout << "inside x = " << point.x <<
      ",y = " << point.y <<
      ",z = " << point.z <<
      std::endl;
    cloud_modifier_input.push_back(point);
  }

  // generate random points within the rectangle
  cloud_modifier_input.reserve(cloud_modifier_input.size() + count_points_outside_rect);
  for (uint32_t i = 0; i < count_points_outside_rect; ++i) {
    auto point = generate_random_point_outside_rect();
    std::cout << "outside x = " << point.x <<
      ",y = " << point.y <<
      ",z = " << point.z <<
      std::endl;
    cloud_modifier_input.push_back(point);
  }
  return cloud_input_ptr;
}

TEST(TestPolygonRemoverNodes, TestRemoveRectangle) {
  rclcpp::init(0, nullptr);
  const uint32_t count_points_within_rect = 50;
  const uint32_t count_points_outside_rect = 100;
  const float32_t bound_x_min = 0.0F;
  const float32_t bound_x_max = 3.0F;
  const float32_t bound_y_min = 0.0F;
  const float32_t bound_y_max = 4.0F;

  auto point_bot_left = make_point_geo(bound_x_min, bound_y_min, 0.0F);
  auto point_bot_right = make_point_geo(bound_x_min, bound_y_max, 0.0F);
  auto point_top_right = make_point_geo(bound_x_max, bound_y_max, 0.0F);
  auto point_top_left = make_point_geo(bound_x_max, bound_y_min, 0.0F);

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("working_mode", "Static");
  params.emplace_back(
    "polygon_vertices",
    std::vector<float32_t>{
    point_bot_left.x, point_bot_left.y,
    point_bot_right.x, point_bot_right.y,
    point_top_right.x, point_top_right.y,
    point_top_left.x, point_top_left.y
  });
  params.emplace_back("will_visualize", false);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  auto node_polygon_remover =
    std::make_shared<autoware::perception::filters::polygon_remover_nodes::PolygonRemoverNode>(
    node_options);

  bool test_completed = false;

  auto pub_ptr_cloud_raw = node_polygon_remover->create_publisher<sensor_msgs::msg::PointCloud2>(
    "points_xyzi",
    rclcpp::QoS(10));

  auto callback_cloud_polygon_removed =
    [&test_completed, &count_points_outside_rect](
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      using CloudView = point_cloud_msg_wrapper::PointCloud2View<PointXYZI>;
      CloudView cloud_view_input(*msg);
      EXPECT_EQ(cloud_view_input.size(), count_points_outside_rect);
      test_completed = true;
    };

  auto sub_ptr_cloud_polygon_removed =
    node_polygon_remover->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud_polygon_removed",
    rclcpp::QoS(rclcpp::KeepLast(10)),
    callback_cloud_polygon_removed);

  auto cloud_ptr_predetermined = generate_cloud_rect_counted(
    count_points_within_rect,
    count_points_outside_rect,
    bound_x_min,
    bound_x_max,
    bound_y_min,
    bound_y_max);

  pub_ptr_cloud_raw->publish(*cloud_ptr_predetermined);

  auto start_time = std::chrono::system_clock::now();
  auto max_test_dur = std::chrono::seconds(1);
  auto timed_out = false;

  while (rclcpp::ok() && !test_completed) {
    rclcpp::spin_some(node_polygon_remover);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    if (std::chrono::system_clock::now() - start_time > max_test_dur) {
      timed_out = true;
      break;
    }
  }
  EXPECT_FALSE(timed_out);
  EXPECT_TRUE(test_completed);
  rclcpp::shutdown();
}
