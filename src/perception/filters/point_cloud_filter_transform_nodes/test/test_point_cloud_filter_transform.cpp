// Copyright 2017-2020 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <apex_test_tools/apex_test_tools.hpp>
#include <gtest/gtest.h>
#include <lidar_integration/lidar_integration.hpp>
#include <point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <velodyne_nodes/velodyne_cloud_node.hpp>
#include <common/types.hpp>
#include <lidar_utils/point_cloud_utils.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

struct TestFilterTransformPC2FilterTransformMode
  : public autoware::perception::filters::point_cloud_filter_transform_nodes
  ::PointCloud2FilterTransformNode
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  TestFilterTransformPC2FilterTransformMode(
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions{})
  : PointCloud2FilterTransformNode
    (
      node_options
    )
  {}

  const PointCloud2 & test_filter_and_transform(const PointCloud2 & msg)
  {
    return filter_and_transform(msg);
  }
};

// FIXME(esteve): this function is copied from test_point_cloud_fusion.hpp
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

// FIXME(esteve): this function is copied from test_point_cloud_fusion.hpp
sensor_msgs::msg::PointCloud2 make_pc(
  std::vector<int32_t> seeds,
  builtin_interfaces::msg::Time stamp)
{
  sensor_msgs::msg::PointCloud2 msg;
  using autoware::common::types::PointXYZIF;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{msg, "base_link"};
  modifier.reserve(seeds.size());

  for (auto seed : seeds) {
    PointXYZIF pt;
    pt.x = seed;
    pt.y = seed;
    pt.z = seed;
    pt.intensity = seed;
    modifier.push_back(pt);
  }

  msg.header.stamp = stamp;

  return msg;
}

class PointCloudFilterTransformIntegration : public ::testing::Test
{
protected:
  using Vlp16Translator = autoware::drivers::velodyne_driver::Vlp16Translator;
  const Vlp16Translator::Config m_vlp_config{600.0F};
  using Transform = geometry_msgs::msg::Transform;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  Transform create_transform(
    std::vector<float64_t> quaternion, std::vector<float64_t>
    translation)
  {
    Transform ret;
    ret.rotation.x = quaternion[0];
    ret.rotation.y = quaternion[1];
    ret.rotation.z = quaternion[2];
    ret.rotation.w = quaternion[3];
    ret.translation.x = translation[0];
    ret.translation.y = translation[1];
    ret.translation.z = translation[2];
    return ret;
  }

  const uint32_t m_cloud_size{55000U};
  const char8_t * const m_ip{"127.0.0.1"};
  const std::chrono::seconds m_init_timeout{std::chrono::seconds(5)};
  const char8_t * const m_frame_id{"base_link"};
  const uint32_t m_sensor_id{0U};
  const float32_t m_start_angle = 0.0;
  const float32_t m_end_angle = 6.28;
  const float32_t m_min_radius = 0.0;
  const float32_t m_max_radius = 10000.0;
  const std::vector<float64_t> m_quaternion{0, 0, 0, 1};
  const std::vector<float64_t> m_translation{0, 0, 0};
  const Transform m_tf = create_transform(m_quaternion, m_translation);
};

bool check_filtered_points(
  const std::vector<sensor_msgs::msg::PointCloud2> points,
  const std::vector<std::vector<autoware::common::types::PointXYZIF>> expected_points,
  const std::string expected_frame_id)
{
  if (points.size() != expected_points.size()) {
    std::cout << "Did not receive expected number of filtered point cloud msgs" << std::endl;
    std::cout << "Expected: " << expected_points.size() << " Actual: " <<
      points.size() << std::endl;
    return false;
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (points[i].header.frame_id != expected_frame_id) {
      std::cout << "Expected frame id " << expected_frame_id << " actual frame id " <<
        points[i].header.frame_id << std::endl;
      return false;
    }
    const uint32_t actual_num_points = points[i].data.size() /
      points[i].point_step;
    if (actual_num_points != expected_points[i].size()) {
      std::cout << "Filtered point cloud msg does not have expected number of points" <<
        std::endl;
      std::cout << "Expected: " << expected_points[i].size() << " Actual: " <<
        actual_num_points << std::endl;
      return false;
    }
    for (uint32_t pc_idx = 0, pt_cnt = 0; pc_idx < points[i].data.size(); pc_idx +=
      points[i].point_step, ++pt_cnt)
    {
      autoware::common::types::PointXYZIF pt;
      //lint -e{925, 9110} Need to convert pointers and use bit for external API NOLINT
      (void)memmove(
        static_cast<void *>(&pt.x),
        static_cast<const void *>(&points[i].data[pc_idx]),
        points[i].point_step);
      if (pt.x != expected_points[i][pt_cnt].x || pt.y != expected_points[i][pt_cnt].y || pt.z !=
        expected_points[i][pt_cnt].z)
      {
        std::cout << "Unexpected point in filtered output" << std::endl;
        return false;
      }
    }
  }
  return true;
}

TEST_F(PointCloudFilterTransformIntegration, CloudBasicTest)
{
  using VelodyneCloudNode = autoware::drivers::velodyne_nodes::VLP16DriverNode;
  using autoware::perception::filters::point_cloud_filter_transform_nodes
  ::PointCloud2FilterTransformNode;
  using lidar_integration::LidarIntegrationPclListener;

  rclcpp::init(0, nullptr);

  // Configuration
  const auto timeout = std::chrono::milliseconds(10);
  const auto max_cycle_time = std::chrono::milliseconds(11);
  const auto port_num = 3445U;
  const auto topic_name = "points_xyzi";
  // Node
  std::vector<rclcpp::Parameter> velodyne_params;
  velodyne_params.emplace_back("ip", m_ip);
  velodyne_params.emplace_back("port", static_cast<int64_t>(port_num));
  velodyne_params.emplace_back("frame_id", m_frame_id);
  velodyne_params.emplace_back("cloud_size", static_cast<int64_t>(m_cloud_size));
  velodyne_params.emplace_back("rpm", static_cast<int>(m_vlp_config.get_rpm()));
  velodyne_params.emplace_back("topic", topic_name);
  rclcpp::NodeOptions velodyne_options = rclcpp::NodeOptions();
  velodyne_options.parameter_overrides(velodyne_params);
  std::shared_ptr<VelodyneCloudNode> velodyne_ptr = std::make_shared<VelodyneCloudNode>(
    "velodyne_cloud_node", velodyne_options);
  std::thread velodyne_node_thread;

  // Node under test
  std::vector<rclcpp::Parameter> params;

  params.emplace_back("start_angle", m_start_angle);
  params.emplace_back("end_angle", m_end_angle);
  params.emplace_back("min_radius", m_min_radius);
  params.emplace_back("max_radius", m_max_radius);
  params.emplace_back("input_frame_id", m_frame_id);
  params.emplace_back("output_frame_id", m_frame_id);
  params.emplace_back("static_transformer.quaternion.x", m_tf.rotation.x);
  params.emplace_back("static_transformer.quaternion.y", m_tf.rotation.y);
  params.emplace_back("static_transformer.quaternion.z", m_tf.rotation.z);
  params.emplace_back("static_transformer.quaternion.w", m_tf.rotation.w);
  params.emplace_back("static_transformer.translation.x", m_tf.translation.x);
  params.emplace_back("static_transformer.translation.y", m_tf.translation.y);
  params.emplace_back("static_transformer.translation.z", m_tf.translation.z);
  params.emplace_back("init_timeout_ms", std::chrono::milliseconds{m_init_timeout}.count());
  params.emplace_back("timeout_ms", 110);
  params.emplace_back("expected_num_publishers", 0);
  params.emplace_back("expected_num_subscribers", 0);
  params.emplace_back("pcl_size", 55000);

  rclcpp::NodeOptions options = rclcpp::NodeOptions().arguments({"points_in:=points_xyzi"});
  options.parameter_overrides(params);
  const auto pc2_filter_ptr = std::make_shared<PointCloud2FilterTransformNode>(options);

  // Listener
  const auto pcl_listen_ptr = std::make_shared<LidarIntegrationPclListener>(
    "points_filtered",
    100,
    29000,
    0.7,  // period tolerance
    0.1F  // size tolerance
  );
  const auto raw_listen_ptr = std::make_shared<LidarIntegrationPclListener>(
    "points_xyzi",
    100,
    29000,
    0.7,  // period tolerance
    0.1F  // size tolerance
  );

  // Spoofer
  const auto spoofer_ptr = std::make_shared<lidar_integration::Vlp16IntegrationSpoofer>(
    m_ip, port_num, m_vlp_config.get_rpm());

  // ===== Run ====== //
  EXPECT_TRUE(
    lidar_integration::lidar_integration_test(
      [&velodyne_node_thread, velodyne_ptr] {
        // Create thread to
        velodyne_node_thread = std::thread {
          [velodyne_ptr] {
            while (rclcpp::ok()) {
              rclcpp::spin(velodyne_ptr);
            }
          }
        };
      },
      [] { /* UdpDriverNode does not allow us to stop it, we need to shutdown */},
      {spoofer_ptr},
      {pcl_listen_ptr, raw_listen_ptr},
      std::chrono::seconds{10},
      {pc2_filter_ptr}
    )
  );

  rclcpp::shutdown();

  // FIXME As mentioned above the UdpDriverNode does not allow us to stop it.
  // Additionally the UdpDriverNode is likely to be blocked in a recvfrom call
  // in UdpDriverNode::get_packet. The easiest way to unblock the node is to
  // send a UDP packet to the given address.
  UdpSender<char> kill_velodyne(m_ip, port_num);
  kill_velodyne.send('D');
  kill_velodyne.send('I');
  kill_velodyne.send('E');

  if (velodyne_node_thread.joinable()) {
    velodyne_node_thread.join();
  }
}

// Add 5 points but 2 of them are outside the desired angle/distance.
TEST_F(PointCloudFilterTransformIntegration, Filter270Radius10) {
  using PointXYZIF = autoware::common::types::PointXYZIF;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using autoware::perception::filters::point_cloud_filter_transform_nodes
  ::PointCloud2FilterTransformNode;
  using lidar_integration::LidarIntegrationPclListener;

  rclcpp::init(0, nullptr);

  std::vector<std::vector<PointXYZIF>> expected_filter_output_points(1);
  PointCloud2 raw_msg;
  using autoware::common::types::PointXYZIF;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZIF> modifier{raw_msg, "lidar_front"};
  modifier.reserve(5);

  PointXYZIF pt;
  pt.x = 1.; pt.y = 2.; pt.z = 3.;
  modifier.push_back(pt);
  expected_filter_output_points[0].push_back(pt);

  pt.x = -1.; pt.y = 2.; pt.z = 3.;
  modifier.push_back(pt);
  expected_filter_output_points[0].push_back(pt);

  pt.x = -1.; pt.y = -2.; pt.z = 3.;
  modifier.push_back(pt);
  expected_filter_output_points[0].push_back(pt);

  pt.x = 1.; pt.y = -2.; pt.z = 3.;  // not within angle_filter
  modifier.push_back(pt);

  pt.x = -1.; pt.y = -11.; pt.z = 3.;  // not within distance filter
  modifier.push_back(pt);

  raw_msg.row_step = raw_msg.width * raw_msg.point_step;

  std::vector<rclcpp::Parameter> params;

  params.emplace_back("start_angle", 0.);
  params.emplace_back("end_angle", 4.712);
  params.emplace_back("min_radius", 0.);
  params.emplace_back("max_radius", 10.);
  params.emplace_back("input_frame_id", "lidar_front");
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("static_transformer.quaternion.x", m_tf.rotation.x);
  params.emplace_back("static_transformer.quaternion.y", m_tf.rotation.y);
  params.emplace_back("static_transformer.quaternion.z", m_tf.rotation.z);
  params.emplace_back("static_transformer.quaternion.w", m_tf.rotation.w);
  params.emplace_back("static_transformer.translation.x", m_tf.translation.x);
  params.emplace_back("static_transformer.translation.y", m_tf.translation.y);
  params.emplace_back("static_transformer.translation.z", m_tf.translation.z);
  params.emplace_back("init_timeout_ms", std::chrono::milliseconds{m_init_timeout}.count());
  params.emplace_back("timeout_ms", 110);
  params.emplace_back("expected_num_publishers", 1);
  params.emplace_back("expected_num_subscribers", 1);
  params.emplace_back("pcl_size", 50);

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  const auto pc2_filter_ptr = std::make_shared<TestFilterTransformPC2FilterTransformMode>(options);

  auto time0 = std::chrono::system_clock::now();
  raw_msg.header.stamp = to_msg_time(time0);
  raw_msg.header.frame_id = "lidar_front";

  std::vector<PointCloud2> output_points(1);
  output_points[0] = pc2_filter_ptr->test_filter_and_transform(raw_msg);
  EXPECT_TRUE(check_filtered_points(output_points, expected_filter_output_points, "base_link"));
  rclcpp::shutdown();
}

// regression test for bug 419
// https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/419
TEST_F(PointCloudFilterTransformIntegration, FilterAndTransformBug419)
{
  rclcpp::init(0, nullptr);

  const std::string input_frame_id = "lidar_front";

  std::vector<rclcpp::Parameter> params;

  params.emplace_back("start_angle", 0.);
  params.emplace_back("end_angle", 4.712);
  params.emplace_back("min_radius", 0.);
  params.emplace_back("max_radius", 10.);
  params.emplace_back("input_frame_id", input_frame_id);
  params.emplace_back("output_frame_id", "base_link");
  params.emplace_back("static_transformer.quaternion.x", m_tf.rotation.x);
  params.emplace_back("static_transformer.quaternion.y", m_tf.rotation.y);
  params.emplace_back("static_transformer.quaternion.z", m_tf.rotation.z);
  params.emplace_back("static_transformer.quaternion.w", m_tf.rotation.w);
  params.emplace_back("static_transformer.translation.x", m_tf.translation.x);
  params.emplace_back("static_transformer.translation.y", m_tf.translation.y);
  params.emplace_back("static_transformer.translation.z", m_tf.translation.z);
  params.emplace_back("init_timeout_ms", std::chrono::milliseconds{m_init_timeout}.count());
  params.emplace_back("timeout_ms", 110);
  params.emplace_back("expected_num_publishers", 1);
  params.emplace_back("expected_num_subscribers", 1);
  params.emplace_back("pcl_size", 5);

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  const auto pc2_filter_ptr = std::make_shared<TestFilterTransformPC2FilterTransformMode>(options);

  auto time0 = std::chrono::system_clock::now();
  auto t0 = to_msg_time(time0);

  auto pc1 = make_pc({1, 2, 3, 4, 5, 6, 7, 8, 9}, t0);
  pc1.header.frame_id = input_frame_id;

  pc2_filter_ptr->test_filter_and_transform(pc1);
  rclcpp::shutdown();
}

int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int32_t ret = 0;
  try {
    ret = RUN_ALL_TESTS();
  } catch (...) {
  }
  return ret;
}
