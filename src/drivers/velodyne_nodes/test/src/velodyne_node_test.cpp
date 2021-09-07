// Copyright 2017-2019 the Autoware Foundation
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

#include <common/types.hpp>
#include <gtest/gtest.h>
#include <velodyne_nodes/velodyne_cloud_node.hpp>
#include <lidar_integration/lidar_integration.hpp>
#include <lidar_integration/udp_sender.hpp>
#include <memory>
#include <thread>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

// This test is just to make sure the alternative constructor doesn't die
TEST(VelodyneNode, Constructor)
{
  rclcpp::init(0, nullptr);

  const auto name = "test_node";

  std::vector<rclcpp::Parameter> velodyne_params;
  velodyne_params.emplace_back("ip", "127.0.0.1");
  velodyne_params.emplace_back("port", 9999);
  velodyne_params.emplace_back("frame_id", "base_link");
  velodyne_params.emplace_back("cloud_size", 10000);
  velodyne_params.emplace_back("rpm", 600);
  velodyne_params.emplace_back("topic", "blah");
  rclcpp::NodeOptions velodyne_options = rclcpp::NodeOptions();
  velodyne_options.parameter_overrides(velodyne_params);

  using VelodyneCloudNode = autoware::drivers::velodyne_nodes::VLP16DriverNode;
  try {
    VelodyneCloudNode v(name, velodyne_options);
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
  }
  EXPECT_NO_THROW(VelodyneCloudNode(name, velodyne_options));

  velodyne_params.pop_back();
  velodyne_options.parameter_overrides(velodyne_params);
  EXPECT_THROW(
    VelodyneCloudNode(name, velodyne_options),
    std::runtime_error
  );

  rclcpp::shutdown();
}

struct VelodyneNodeTestParam
{
  uint32_t reserved_size;
  uint32_t expected_size;
  float32_t expected_period_ms;
  bool8_t is_cloud;
};  // VelodyneNodeTestParam

class VelodyneNodeIntegration : public ::testing::TestWithParam<VelodyneNodeTestParam>
{
};  // class velodyne_node_integration

TEST_P(VelodyneNodeIntegration, Test)
{
  rclcpp::init(0, nullptr);

  const auto param = GetParam();
  // Configuration
  const auto name = "test_node";
  const auto ip = "127.0.0.1";
  const auto port = 3555U;
  const auto frame_id = "base_link";
  const auto topic = "points_xyzi";
  const auto runtime = std::chrono::seconds(10);
  using autoware::drivers::velodyne_driver::Vlp16Translator;
  const auto config = Vlp16Translator::Config{600.0F};
  std::vector<rclcpp::Parameter> velodyne_params;
  velodyne_params.emplace_back("ip", ip);
  velodyne_params.emplace_back("port", static_cast<int64_t>(port));
  velodyne_params.emplace_back("frame_id", frame_id);
  velodyne_params.emplace_back("cloud_size", static_cast<int64_t>(param.reserved_size));
  velodyne_params.emplace_back("rpm", static_cast<int>(config.get_rpm()));
  velodyne_params.emplace_back("topic", topic);
  rclcpp::NodeOptions velodyne_options = rclcpp::NodeOptions();
  velodyne_options.parameter_overrides(velodyne_params);

  // Node
  using VelodyneCloudNode = autoware::drivers::velodyne_nodes::VLP16DriverNode;
  std::shared_ptr<VelodyneCloudNode> nd_ptr = std::make_shared<VelodyneCloudNode>(
    name, velodyne_options);
  std::thread velodyne_node_thread;

  // Listener
  using lidar_integration::LidarIntegrationListener;
  std::shared_ptr<LidarIntegrationListener> listen_ptr;
  using lidar_integration::LidarIntegrationPclListener;
  listen_ptr = std::make_shared<LidarIntegrationPclListener>(
    topic,
    param.expected_period_ms,
    param.expected_size,
    0.7,  // period tolerance
    0.1F  // size tolerance
  );

  // Spoofer
  const auto spoofer_ptr =
    std::make_shared<lidar_integration::Vlp16IntegrationSpoofer>(ip, port, config.get_rpm());

  // ===== Run ====== //
  EXPECT_TRUE(
    lidar_integration::lidar_integration_test(
      [&velodyne_node_thread, nd_ptr] {
        // Create thread to
        velodyne_node_thread = std::thread {[nd_ptr] {
            while (rclcpp::ok()) {
              rclcpp::spin(nd_ptr);
            }
          }};
      },
      [] { /* UdpDriverNode does not allow us to stop it, we need to shutdown */},
      {spoofer_ptr}, {listen_ptr}));

  rclcpp::shutdown();

  // FIXME As mentioned above the UdpDriverNode does not allow us to stop it.
  // Additionally the UdpDriverNode is likely to be blocked in a recvfrom call
  // in UdpDriverNode::get_packet. The easiest way to unblock the node is to
  // send a UDP packet to the given address.
  UdpSender<char> kill_velodyne(ip, port);
  kill_velodyne.send('D');
  kill_velodyne.send('I');
  kill_velodyne.send('E');

  if (velodyne_node_thread.joinable()) {
    velodyne_node_thread.join();
  }
}

INSTANTIATE_TEST_CASE_P(
  Cloud,
  VelodyneNodeIntegration,
  // cppcheck-suppress syntaxError
  ::testing::Values(VelodyneNodeTestParam{55000U, 30000U, 100.0F, true}), );

INSTANTIATE_TEST_CASE_P(
  HalfCloud,
  VelodyneNodeIntegration,
  // cppcheck-suppress syntaxError
  ::testing::Values(VelodyneNodeTestParam{10700U, 10700U, 50.0F, true}), );
