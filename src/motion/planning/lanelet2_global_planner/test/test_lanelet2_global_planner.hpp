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

#ifndef TEST_LANELET2_GLOBAL_PLANNER_HPP_
#define TEST_LANELET2_GLOBAL_PLANNER_HPP_

#include <gtest/gtest.h>
#include <lanelet2_global_planner/lanelet2_global_planner.hpp>
#include <common/types.hpp>
#include <experimental/filesystem>
#include <string>
#include <memory>
#include <vector>

using autoware::common::types::float64_t;
using autoware::common::types::bool8_t;
using autoware::motion::planning::lanelet2_global_planner::Lanelet2GlobalPlannerNode;

class TestGlobalPlanner : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;

    node_ptr = std::make_shared<Lanelet2GlobalPlannerNode>(node_options);
    std::string root_folder = std::string(std::experimental::filesystem::current_path());
    std::string file_path = std::string("/test/map_data/mapping_example_pk.osm");
    std::string file = root_folder + file_path;
    float64_t lat = 51.502091;
    float64_t lon = -0.08719;
    float64_t alt = 39.0144;
    node_ptr->load_osm_map(file, lat, lon, alt);
    node_ptr->parse_lanelet_element();
  }
  void TearDown()
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<Lanelet2GlobalPlannerNode> node_ptr;
};

TEST(TestFunction, Point3d_copy)
{
  auto assign_point3d = [](lanelet::Point3d & pcopy)
    {
      lanelet::Point3d p(lanelet::utils::getId(), 2.0, 4.0, 6.0);
      pcopy = p;
    };

  lanelet::Point3d point3d;
  assign_point3d(point3d);
  ASSERT_DOUBLE_EQ(2.0, point3d.x());
  ASSERT_DOUBLE_EQ(4.0, point3d.y());
  ASSERT_DOUBLE_EQ(6.0, point3d.z());
}

TEST(TestFunction, Point3d_eigen)
{
  lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
  Eigen::Vector3d pd = p1.basicPoint() - p2.basicPoint();
  Eigen::Vector3d pd2 = pd.array().square();
  float64_t dist = std::sqrt(pd2.x() + pd2.y() + pd2.z());
  float64_t actual = std::sqrt(std::pow(p2.x() - p1.x(), 2) +
      std::pow(p2.y() - p1.y(), 2) +
      std::pow(p2.z() - p1.z(), 2));
  ASSERT_DOUBLE_EQ(actual, dist);
}

TEST(TestFunction, Point3d_mean)
{
  lanelet::Point3d p1(lanelet::utils::getId(), 1, 4, 7);
  lanelet::Point3d p2(lanelet::utils::getId(), 2, 5, 8);
  lanelet::Point3d p3(lanelet::utils::getId(), 3, 6, 9);
  lanelet::LineString3d ls(lanelet::utils::getId(), {p1, p2, p3});
  size_t num_points = ls.size();
  // sum x,y,z points
  float64_t mean_x = 0.0;
  float64_t mean_y = 0.0;
  float64_t mean_z = 0.0;
  std::for_each(ls.begin(), ls.end(), [&](lanelet::Point3d p)
    {
      mean_x += p.x() / static_cast<float64_t>(num_points);
      mean_y += p.y() / static_cast<float64_t>(num_points);
      mean_z += p.z() / static_cast<float64_t>(num_points);
    });
  ASSERT_DOUBLE_EQ(2.0, mean_x);
  ASSERT_DOUBLE_EQ(5.0, mean_y);
  ASSERT_DOUBLE_EQ(8.0, mean_z);
}

TEST_F(TestGlobalPlanner, test_find_parking)
{
  // Find parking id from point
  // parking id: 5830 location: 20.7607, -10.2697, 15.6 near_roads: 479054
  lanelet::Point3d position(lanelet::utils::getId(), 20.7607, -10.2697, 15.6);
  lanelet::Id parking_id = node_ptr->find_nearparking_from_point(position);
  EXPECT_EQ(parking_id, 5830);

  // Compute parking centre
  lanelet::Point3d p1;
  bool8_t ret = false;
  ret = node_ptr->compute_parking_center(parking_id, p1);
  EXPECT_TRUE(ret);

  // near road id
  lanelet::Id near_road = node_ptr->find_nearroute_from_parking(parking_id);
  EXPECT_EQ(near_road, 479054);
}

TEST_F(TestGlobalPlanner, test_find_route)
{
  // find lane cad_id to lane id
  // id:6895 -> cad_id 445864
  // id:6944 -> cad_id 448826
  lanelet::Id lane_cad_id_start = 445864;
  lanelet::Id lane_cad_id_end = 448826;
  lanelet::Id expect_id_start = 6895;
  lanelet::Id expect_id_end = 6944;
  lanelet::Id lane_start = node_ptr->find_lane_id(lane_cad_id_start);
  lanelet::Id lane_end = node_ptr->find_lane_id(lane_cad_id_end);
  EXPECT_EQ(lane_start, expect_id_start);
  EXPECT_EQ(lane_end, expect_id_end);

  // plan a route from start to end lane id
  std::vector<lanelet::Id> route_id = node_ptr->get_lane_route(lane_start, lane_end);
  EXPECT_GT(route_id.size(), 0u);
}

TEST_F(TestGlobalPlanner, test_plan_route)
{
  // from bay 5830: (20.7607, -10.2697, 15.6)
  // to bay: 6139: (29.0561, 5.7876, 16.9)
  // lane id: 7455, 8389, 7441, 8626, 7539, 8413, 7504
  lanelet::Point3d start(lanelet::utils::getId(), 20.76, -10.26, 15.60);
  lanelet::Point3d end(lanelet::utils::getId(), 29.05, 5.78, 16.90);
  std::vector<lanelet::Id> route;
  bool8_t ret = node_ptr->plan_route(start, end, route);
  EXPECT_TRUE(ret);
}

TEST_F(TestGlobalPlanner, test_p2p_distance)
{
  lanelet::Point3d p1(lanelet::utils::getId(), 1.0, 2.0, 3.0);
  lanelet::Point3d p2(lanelet::utils::getId(), 4.0, 5.0, 6.0);
  float64_t dist = node_ptr->p2p_euclidean(p1, p2);
  float64_t actual = std::sqrt(std::pow(p2.x() - p1.x(), 2) +
      std::pow(p2.y() - p1.y(), 2) +
      std::pow(p2.z() - p1.z(), 2));
  ASSERT_DOUBLE_EQ(actual, dist);
}

TEST_F(TestGlobalPlanner, test_lanes_str2num)
{
  std::string str = "[u'429933', u'430462']";
  std::vector<lanelet::Id> num = node_ptr->str2num_lanes(str);
  ASSERT_EQ(num[0], 429933);
  ASSERT_EQ(num[1], 430462);
}
#endif  // TEST_LANELET2_GLOBAL_PLANNER_HPP_
