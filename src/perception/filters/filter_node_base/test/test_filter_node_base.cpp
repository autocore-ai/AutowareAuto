// Copyright 2021 Tier IV, Inc.
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


#include <memory>
#include <vector>

#include "filter_node_base/filter_node_base.hpp"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "lidar_utils/point_cloud_utils.hpp"


using float32_t = autoware::common::types::float32_t;
using FilterNodeBase = autoware::perception::filters::filter_node_base::FilterNodeBase;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

/* \class MockFilterNodeBase
 * \brief This class implements the FilterNodeBase to test for correct inheritence
 */
class MockFilterNodeBase : public FilterNodeBase
{
public:
  explicit MockFilterNodeBase(const rclcpp::NodeOptions & options)
  : FilterNodeBase("test_filter_node", options) {}

  ~MockFilterNodeBase() {}

  MOCK_METHOD(
    void, filter,
    (const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output),
    (override));
  MOCK_METHOD(
    rcl_interfaces::msg::SetParametersResult, get_node_parameters,
    (const std::vector<rclcpp::Parameter>&p), (override));
};

/* \class TestFilterNodeBase
 * \brief This class sets up the test environment
 */
class TestFilterNodeBase : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    // Generate parameters
    std::vector<rclcpp::Parameter> params;
    params.emplace_back("max_queue_size", 5);

    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(params);

    // Create instance of the TestFilter child class
    mock_filter_node_base = std::make_shared<MockFilterNodeBase>(node_options);
  }

  std::shared_ptr<MockFilterNodeBase> mock_filter_node_base;
};

using ::testing::_;
using ::testing::AtLeast;

/* \brief Create a dummy point cloud for publishing */
inline void create_dummy_cloud(sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<float32_t> seeds = {0.0, 0.0, 0.0};
  autoware::common::lidar_utils::init_pcl_msg(cloud, "base_link", seeds.size());

  uint32_t pidx = 0;
  for (auto seed : seeds) {
    autoware::common::types::PointXYZIF pt;
    pt.x = seed;
    pt.y = seed;
    pt.z = seed;
    pt.intensity = seed;
    autoware::common::lidar_utils::add_point_to_cloud(cloud, pt, pidx);
  }
}

TEST_F(TestFilterNodeBase, test_filter) {
  // Create dummy point cloud
  sensor_msgs::msg::PointCloud2 cloud;
  create_dummy_cloud(cloud);

  auto cloud_pub = mock_filter_node_base->create_publisher<sensor_msgs::msg::PointCloud2>(
    "input",
    rclcpp::QoS(10));

  // Check that the filter method in the MockFilterNodeBase class has been called at least once
  EXPECT_CALL(*mock_filter_node_base, filter(_, _)).Times(AtLeast(1));
  // Publish cloud
  cloud_pub->publish(cloud);
  rclcpp::spin_some(mock_filter_node_base);
}

TEST_F(TestFilterNodeBase, test_parameters) {
  // Set up parameter client
  auto client = std::make_shared<rclcpp::SyncParametersClient>(mock_filter_node_base);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  const std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("max_queue_size", 10),
  };

  // Check that the get_node_parameters method in the MockFilterNodeBase class has been called at
  // least once
  EXPECT_CALL(*mock_filter_node_base, get_node_parameters(_));
  // Set new parameters
  client->set_parameters(parameters);
  rclcpp::spin_some(mock_filter_node_base);
}
