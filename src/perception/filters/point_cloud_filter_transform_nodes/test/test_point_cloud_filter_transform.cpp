// Copyright 2017-2020 Apex.AI, Inc.
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
#include <velodyne_node/velodyne_cloud_node.hpp>
#include <common/types.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

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

class point_cloud_filter_transform_integration : public ::testing::Test
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

class PointCloudFilterPclValidationTester : public rclcpp::Node
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

public:
//   PointCloudFilterPclValidationTester(
//     const std::string & publish_topic_name, const std::string &
//     subscribe_topic_name)
//   : Node("pcl_listener"),
//     m_sub_filtered_points
//       {create_subscription<PointCloud2>(subscribe_topic_name, rclcpp::QoS{10})},
//     m_pub_raw_points{create_publisher<PointCloud2>(publish_topic_name, rclcpp::QoS{50})}
//   {
//   }

//   bool check_filtered_points(
//     std::vector<std::vector<autoware::common::types::PointXYZIF>> expected_points,
//     const std::string expected_frame_id)
//   {
//     if (m_filtered_pcl_msgs.size() != expected_points.size()) {
//       std::cout << "Did not receive expected number of filtered point cloud msgs" << std::endl;
//       std::cout << "Expected: " << expected_points.size() << " Actual: " <<
//         m_filtered_pcl_msgs.size() << std::endl;
//       return false;
//     }
//     for (std::size_t i = 0; i < m_filtered_pcl_msgs.size(); ++i) {
//       if (m_filtered_pcl_msgs[i].header.frame_id != expected_frame_id) {
//         std::cout << "Expected frame id " << expected_frame_id << " actual frame id " <<
//           m_filtered_pcl_msgs[i].header.frame_id << std::endl;
//         return false;
//       }
//       const uint32_t actual_num_points = m_filtered_pcl_msgs[i].data.size() /
//         m_filtered_pcl_msgs[i].point_step;
//       if (actual_num_points != expected_points[i].size()) {
//         std::cout << "Filtered point cloud msg does not have expected number of points" <<
//           std::endl;
//         std::cout << "Expected: " << expected_points[i].size() << " Actual: " <<
//           actual_num_points << std::endl;
//         return false;
//       }
//       for (uint32_t pc_idx = 0, pt_cnt = 0; pc_idx < m_filtered_pcl_msgs[i].data.size();
//            pc_idx += m_filtered_pcl_msgs[i].point_step, ++pt_cnt)
//       {
//         autoware::common::types::PointXYZIF pt;
//         //lint -e{925, 9110} Need to convert pointers and use bit for external API NOLINT
//         (void)memmove(
//           static_cast<void *>(&pt.x),
//           static_cast<const void *>(&m_filtered_pcl_msgs[i].data[pc_idx]),
//           m_filtered_pcl_msgs[i].point_step);
//         if (pt.x != expected_points[i][pt_cnt].x || pt.y != expected_points[i][pt_cnt].y ||
//             pt.z != expected_points[i][pt_cnt].z)
//         {
//           std::cout << "Unexpected point in filtered output" << std::endl;
//           return false;
//         }
//       }
//     }
//     return true;
//   }

  void process_pcl_data()
  {
  }

  // bool collect_pcl_data(
  //   const std::chrono::milliseconds timeout,
  //   const uint32_t expected_num_of_msgs)
  // {
  //   bool retval = true;
  //   const auto start = std::chrono::steady_clock::now();

  //   while (m_filtered_pcl_msgs.size() < expected_num_of_msgs) {
  //     try {
  //       (void) m_waitset.wait(std::chrono::milliseconds(1000));
  //       const auto recv1 = m_sub_filtered_points->take();
  //       for (const auto msg : recv1) {
  //         // Workaround for https://github.com/ros2/rcl/pull/356
  //         if (msg.info().valid()) {
  //           m_filtered_pcl_msgs.emplace_back(msg.data());
  //         }
  //       }
  //     } catch (const std::exception & e) {
  //       std::cout << "waitset timedout" << std::endl;
  //     }

  //     if (std::chrono::steady_clock::now() - start >= timeout) {
  //       retval = false;
  //       break;
  //     }
  //   }
  //   return retval;
  // }

  std::vector<PointCloud2> m_filtered_pcl_msgs;
  std::shared_ptr<rclcpp::Subscription<PointCloud2>> m_sub_filtered_points;
  std::shared_ptr<rclcpp::Publisher<PointCloud2>> m_pub_raw_points;
  // rclcpp::Waitset<1> m_waitset;
};


// TEST_F(point_cloud_filter_transform_integration, cloud_basic_test)
// {
//   // Configuration
//   const auto timeout = std::chrono::milliseconds(10);
//   const auto max_cycle_time = std::chrono::milliseconds(11);
//   const auto port_num = 3445U;
//   // Node
//   using autoware::drivers::velodyne_node::VelodyneCloudNode;
//   const auto velodyne_ptr = std::make_shared<VelodyneCloudNode>(
//     "velodyne_cloud_node",
//     m_ip,
//     port_num,
//     timeout,
//     m_init_timeout,
//     max_cycle_time,
//     m_frame_id,
//     m_cloud_size,
//     m_vlp_config,
//     1U);

//   // Node under test
//   const auto period = std::chrono::milliseconds(200);
//   using autoware::perception::filters::point_cloud_filter_transform_nodes
//   ::PointCloud2FilterTransformNode;
//   const auto pc2_filter_ptr = std::make_shared<PointCloud2FilterTransformNode>(
//     "point_cloud_filter_transform_node",
//     std::chrono::milliseconds{200},
//     "",
//     m_init_timeout,
//     std::chrono::milliseconds(110),
//     m_frame_id,
//     m_frame_id,
//     "points_raw",
//     "points_filtered",
//     m_start_angle,
//     m_end_angle,
//     m_min_radius,
//     m_max_radius,
//     m_tf,
//     "PointCloud2FilterTransformNode_logger",
//     55000U,
//     0U,
//     0U);

//   // Listener
//   using lidar_integration::LidarIntegrationPclListener;
//   const auto pcl_listen_ptr = std::make_shared<LidarIntegrationPclListener>(
//     "points_filtered",
//     100,
//     29000,
//     0.7,  // period tolerance
//     0.1F  // size tolerance
//   );
//   const auto raw_listen_ptr = std::make_shared<LidarIntegrationPclListener>(
//     "points_raw",
//     100,
//     29000,
//     0.7,  // period tolerance
//     0.1F  // size tolerance
//   );

//   // Spoofer
//   const auto spoofer_ptr = std::make_shared<lidar_integration::Vlp16IntegrationSpoofer>(
//     m_ip, port_num, m_vlp_config.get_rpm());

//   // ===== Run ====== //
//   EXPECT_TRUE(
//     lidar_integration::lidar_integration_test(
//       {velodyne_ptr, pc2_filter_ptr},
//       {spoofer_ptr},
//       {pcl_listen_ptr, raw_listen_ptr}
//     )
//   );
// }

// // Add 5 points but 2 of them are outside the desired angle/distance.
// TEST_F(point_cloud_filter_transform_integration, filter_270_radius_10) {
//   using PointXYZIF = autoware::common::types::PointXYZIF;
//   using PointCloud2 = sensor_msgs::msg::PointCloud2;

//   std::vector<std::vector<PointXYZIF>> expected_filter_output_points(1);
//   PointCloud2 raw_msg;
//   autoware::common::lidar_utils::init_pcl_msg(raw_msg, "lidar_front", 5);

//   PointXYZIF pt;
//   pt.x = 1.; pt.y = 2.; pt.z = 3.;
//   (void)autoware::common::lidar_utils::add_point_to_cloud(raw_msg, pt);
//   expected_filter_output_points[0].push_back(pt);
//   pt.x = -1.; pt.y = 2.; pt.z = 3.;
//   (void)autoware::common::lidar_utils::add_point_to_cloud(raw_msg, pt);
//   expected_filter_output_points[0].push_back(pt);
//   pt.x = -1.; pt.y = -2.; pt.z = 3.;
//   (void)autoware::common::lidar_utils::add_point_to_cloud(raw_msg, pt);
//   expected_filter_output_points[0].push_back(pt);
//   pt.x = 1.; pt.y = -2.; pt.z = 3.;  // not within angle_filter
//   (void)autoware::common::lidar_utils::add_point_to_cloud(raw_msg, pt);
//   pt.x = -1.; pt.y = -11.; pt.z = 3.;  // not within distance filter
//   (void)autoware::common::lidar_utils::add_point_to_cloud(raw_msg, pt);

//   raw_msg.row_step = raw_msg.width * raw_msg.point_step;

//   const std::string raw_topic_name{"points_raw"};
//   const std::string filtered_topic_name{"points_filtered"};
//   PointCloudFilterPclValidationTester tester(raw_topic_name, filtered_topic_name);

//   using autoware::perception::filters::point_cloud_filter_transform_nodes
//   ::PointCloud2FilterTransformNode;
//   const float32_t start_angle = 0.;
//   const float32_t end_angle = 4.712;
//   const float32_t min_radius = 0.;
//   const float32_t max_radius = 10.;
//   const auto pc2_filter_ptr = std::make_shared<PointCloud2FilterTransformNode>(
//     "point_cloud_filter_transform_node",
//     std::chrono::milliseconds{200},
//     "",
//     m_init_timeout,
//     std::chrono::milliseconds(110),
//     "lidar_front",
//     "base_link",
//     raw_topic_name,
//     filtered_topic_name,
//     start_angle,
//     end_angle,
//     min_radius,
//     max_radius,
//     m_tf,
//     "PointCloud2FilterTransformNode_logger",
//     50U,
//     1U,
//     1U);
//   pc2_filter_ptr->run();

//   tester.m_pub_raw_points->wait_for_matched(1U, std::chrono::milliseconds(500U));
//   tester.m_sub_filtered_points->wait_for_matched(1U, std::chrono::milliseconds(500U));
//   tester.m_pub_raw_points->publish(raw_msg);

//   const auto time_out = std::chrono::milliseconds(3000);
//   EXPECT_TRUE(tester.collect_pcl_data(time_out, 1));

//   pc2_filter_ptr->stop();
//   pc2_filter_ptr->join();

//   EXPECT_TRUE(tester.check_filtered_points(expected_filter_output_points, "base_link"));
// }

struct TestFilterTransformPC2FilterTransformMode
  : public autoware::perception::filters::point_cloud_filter_transform_nodes
  ::PointCloud2FilterTransformNode
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  explicit TestFilterTransformPC2FilterTransformMode(
    const rclcpp::NodeOptions & node_options)
  : PointCloud2FilterTransformNode(node_options)
  {}

  const PointCloud2 & test_filter_and_transform(const PointCloud2 & msg)
  {
    return filter_and_transform(msg);
  }
};

// regression test for bug 419
// https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/419
TEST_F(point_cloud_filter_transform_integration, filter_and_transform_bug419)
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
