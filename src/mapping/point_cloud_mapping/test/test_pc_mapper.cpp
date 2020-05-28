// Copyright 2020 Apex.AI, Inc.
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include "test_pc_mapper.hpp"
#include <point_cloud_mapping/point_cloud_mapper.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <gtest/gtest.h>
#include <string>
#include <memory>
#include <utility>
#include <vector>

using autoware::mapping::point_cloud_mapping::PlainPointCloudMap;
using autoware::mapping::point_cloud_mapping::MapUpdateType;
using autoware::mapping::point_cloud_mapping::PCMapperTestContext;
using autoware::mapping::point_cloud_mapping::PrefixGeneratorBase;
using autoware::mapping::point_cloud_mapping::MockLocalizer;
using autoware::mapping::point_cloud_mapping::PointCloudMapper;
using autoware::mapping::point_cloud_mapping::CapacityTrigger;
using autoware::mapping::point_cloud_mapping::PCLCloud;
using autoware::mapping::point_cloud_mapping::check_pc_equal;
using autoware::mapping::point_cloud_mapping::MockLocalizerSummary;
using autoware::mapping::point_cloud_mapping::Cloud;

class PCMapperTest : public PCMapperTestContext, public ::testing::Test {};

class MockPrefixGenerator : public PrefixGeneratorBase<MockPrefixGenerator>
{
public:
  std::string get_(const std::string & base_prefix) const noexcept
  {
    return base_prefix;
  }
};


TEST_F(PCMapperTest, core) {
  const std::string fn_prefix{"pc_mapper_test"};

  auto register_and_check = [](const auto & pc, const auto & tf, auto update, auto & mapper) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose_out;
      const auto sum = mapper.register_measurement(pc, geometry_msgs::msg::TransformStamped{},
          pose_out);
      EXPECT_EQ(sum.map_update_summary.update_type, update);
      EXPECT_EQ(sum.map_update_summary.num_added_pts, pc.width);
      EXPECT_EQ(sum.map_increment->width, pc.width);
      EXPECT_DOUBLE_EQ(tf.transform.translation.x, pose_out.pose.pose.position.x);
      EXPECT_DOUBLE_EQ(tf.transform.translation.y, pose_out.pose.pose.position.y);
      EXPECT_DOUBLE_EQ(tf.transform.translation.z, pose_out.pose.pose.position.z);
      EXPECT_DOUBLE_EQ(tf.transform.rotation.x, pose_out.pose.pose.orientation.x);
      EXPECT_DOUBLE_EQ(tf.transform.rotation.y, pose_out.pose.pose.orientation.y);
      EXPECT_DOUBLE_EQ(tf.transform.rotation.z, pose_out.pose.pose.orientation.z);
      EXPECT_DOUBLE_EQ(tf.transform.rotation.w, pose_out.pose.pose.orientation.w);
    };
  {
    auto localizer = std::make_unique<MockLocalizer>(m_tf1, m_tf2);
    PlainPointCloudMap pc_map(m_pc1.width + m_pc2.width + 10U, map_frame);
    geometry_msgs::msg::PoseWithCovarianceStamped dummy;
    PointCloudMapper<MockLocalizer, PlainPointCloudMap,
      CapacityTrigger, MockPrefixGenerator>
    mapper{fn_prefix,
      std::move(pc_map), std::move(localizer), map_frame};
    geometry_msgs::msg::TransformStamped identity{};
    identity.transform.rotation.w = 1.0;
    // First pc is centered on the map.
    register_and_check(m_pc0, identity, MapUpdateType::NEW, mapper);
    register_and_check(m_pc1, m_tf1, MapUpdateType::UPDATE, mapper);
    register_and_check(m_pc2, m_tf2, MapUpdateType::UPDATE, mapper);
  }
  PCLCloud final_map;
  const auto file_name = fn_prefix + ".pcd";
  pcl::io::loadPCDFile(file_name, final_map);

  check_pc_equal(m_expected_map, final_map);
  remove(file_name.c_str());
}

///////////////////////////////////////////////

constexpr char const * PCMapperTestContext::frame0;
constexpr char const * PCMapperTestContext::frame1;
constexpr char const * PCMapperTestContext::frame2;
constexpr char const * PCMapperTestContext::map_frame;

MockLocalizer::MockLocalizer(
  const geometry_msgs::msg::TransformStamped & tf1,
  const geometry_msgs::msg::TransformStamped & tf2)
{
  auto tf_to_pose = [](const geometry_msgs::msg::TransformStamped & tf,
      decltype(m_fixed_estimate1) & pose) {
      pose.header.frame_id = tf.header.frame_id;
      auto t = tf.transform.translation;
      auto r = tf.transform.rotation;
      pose.pose.pose.position.set__x(t.x).set__y(t.y).set__z(t.z);
      pose.pose.pose.orientation.set__x(r.x).set__y(r.y).set__z(r.z).set__w(r.w);
    };

  tf_to_pose(tf1, m_fixed_estimate1);
  tf_to_pose(tf2, m_fixed_estimate2);

  set_map_valid();
}


/// Get the frame id of the current map.
const std::string & MockLocalizer::map_frame_id() const noexcept
{
  return m_map_frame;
}

/// Get the timestamp of the current map.
std::chrono::system_clock::time_point MockLocalizer::map_stamp() const noexcept
{
  return std::chrono::system_clock::now();
}

/// `set_map` implementation.
void MockLocalizer::set_map_impl(const Cloud &) {}

/// `insert_to_map` implementation
void MockLocalizer::insert_to_map_impl(const Cloud &) {}

MockLocalizerSummary MockLocalizer::register_measurement_impl(
  const Cloud & msg,
  const MockLocalizer::Transform &,
  MockLocalizer::PoseWithCovarianceStamped & pose_out)
{
  pose_out = (msg.header.frame_id == PCMapperTestContext::frame1) ?
    m_fixed_estimate1 : m_fixed_estimate2;
  return MockLocalizerSummary{};
}

PCMapperTestContext::PCMapperTestContext()
{
  using Point = common::types::PointXYZIF;

  auto make_tf = [](auto x, auto y, auto z, auto r, auto p, auto yaw, const auto & frame) {
      geometry_msgs::msg::TransformStamped tf_stamped;
      auto & tf = tf_stamped.transform;
      tf.translation.set__x(x).set__y(y).set__z(z);
      tf2::Quaternion q;
      q.setRPY(r, p, yaw);
      tf.rotation.set__x(q.x()).set__y(q.y()).set__z(q.z()).set__w(q.w());
      tf_stamped.header.frame_id = map_frame;
      tf_stamped.child_frame_id = frame;
      return tf_stamped;
    };

  auto make_pc = [](auto & pc, const auto & frame, auto num_pts, auto diff) {
      auto make_base_pts = [](std::size_t num_pts, float_t diff) {
          std::vector<Point> pts;
          for (auto i = 1U; i < num_pts + 1U; ++i) {
            pts.push_back({0.0, static_cast<float_t>(i) * diff, 0.0, 0.0});
          }
          return pts;
        };
      const auto pts1 = make_base_pts(num_pts, diff);
      common::lidar_utils::init_pcl_msg(pc, frame, pts1.size());
      auto pc_idx = 0U;
      for (const auto & pt : pts1) {
        common::lidar_utils::add_point_to_cloud(pc, pt, pc_idx);
      }
    };

  make_pc(m_pc0, frame0, 2U, 4.2F);
  make_pc(m_pc1, frame1, 3U, 2.0F);
  make_pc(m_pc2, frame2, 7U, 1.5F);

  m_tf1 = make_tf(1, 2, 3, 0.1, 1.4, -0.7, frame1);
  m_tf2 = make_tf(5, 8, 10, -1.1, 0.0, 0.4, frame2);

  make_map();
}

void PCMapperTestContext::make_map()
{
  Cloud pc1_map, pc2_map;
  common::lidar_utils::init_pcl_msg(pc1_map, map_frame, m_pc1.width);
  common::lidar_utils::init_pcl_msg(pc2_map, map_frame, m_pc2.width);
  tf2::doTransform(m_pc1, pc1_map, m_tf1);
  tf2::doTransform(m_pc2, pc2_map, m_tf2);

  append_to_pcl(m_pc0, m_expected_map);  // first message should be on the map frame.
  append_to_pcl(pc1_map, m_expected_map);
  append_to_pcl(pc2_map, m_expected_map);
}

void autoware::mapping::point_cloud_mapping::append_to_pcl(const Cloud & pc, PCLCloud & res)
{
  const auto check_not_end = [](const auto & its) {
      return std::all_of(its.cbegin(), its.cend(), [](const auto & it) {return it != it.end();});
    };

  std::array<sensor_msgs::PointCloud2ConstIterator<float_t>, 4U> pc_its = {
    sensor_msgs::PointCloud2ConstIterator<float_t>(pc, "x"),
    sensor_msgs::PointCloud2ConstIterator<float_t>(pc, "y"),
    sensor_msgs::PointCloud2ConstIterator<float_t>(pc, "z"),
    sensor_msgs::PointCloud2ConstIterator<float_t>(pc, "intensity")
  };

  auto pc_idx = 0U;
  for (; check_not_end(pc_its); ++pc_idx) {
    // Transfer the point from the observation to the map and advance the iterators.
    pcl::PointXYZI pt;
    pt.x = *pc_its[0];
    pt.y = *pc_its[1];
    pt.z = *pc_its[2];
    pt.intensity = *pc_its[3];
    res.push_back(pt);
    // Advance the iterators.
    for (auto idx = 0U; idx < 4U; ++idx) {
      ++pc_its[idx];
    }
  }
}
