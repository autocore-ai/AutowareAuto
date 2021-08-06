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
#include <string>
#include <vector>

#include "fake_test_node/fake_test_node.hpp"
#include "filter_node_base/filter_node_base.hpp"
#include "point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp"

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "lidar_utils/point_cloud_utils.hpp"

namespace
{
using float32_t = autoware::common::types::float32_t;
using FilterNodeTest = autoware::tools::testing::FakeTestNode;
using FilterNodeBase = autoware::perception::filters::filter_node_base::FilterNodeBase;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Eq;
using ::testing::Invoke;

/* \class MockFilterNodeBase
 * \brief This class implements the FilterNodeBase to test for correct inheritence
 */
class MockFilterNodeBase : public FilterNodeBase
{
public:
  explicit MockFilterNodeBase(const rclcpp::NodeOptions & options)
  : FilterNodeBase("test_filter_node", options)
  {
    test_param_1_ = declare_parameter("test_param_1").get<double>();
    test_param_2_ = declare_parameter("test_param_2").get<std::string>();

    this->set_param_callback();
  }

  ~MockFilterNodeBase() {}

  MOCK_METHOD(
    void, filter,
    (const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output),
    (override));

  MOCK_METHOD(
    rcl_interfaces::msg::SetParametersResult, get_node_parameters,
    (const std::vector<rclcpp::Parameter>&p), (override));

  void DelegateToFake()
  {
    ON_CALL(*this, get_node_parameters(_))
    .WillByDefault(Invoke(this, &MockFilterNodeBase::mock_get_node_parameters));

    ON_CALL(*this, filter)
    .WillByDefault(
      [this](const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output) {
        mock_filter(input, output);
      });
  }

  // Parameters used by the class
  double test_param_1_;
  std::string test_param_2_;

private:
  // Implement the mock_get_node_parameters method to be called instead of the virtual
  // get_node_parameters method
  rcl_interfaces::msg::SetParametersResult mock_get_node_parameters(
    const std::vector<rclcpp::Parameter> & p)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    {
      using namespace autoware::perception::filters::filter_node_base; // NOLINT

      get_param(p, "test_param_1", test_param_1_);
      get_param(p, "test_param_2", test_param_2_);
    }

    return result;
  }

  // Implement the filter method to just republish the cloud
  void mock_filter(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output)
  {
    // Pass through filter
    output = input;
  }
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
    params.emplace_back("test_param_1", 0.5);
    params.emplace_back("test_param_2", "frame_2");

    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides(params);

    // Create instance of the TestFilter child class
    mock_filter_node_base = std::make_shared<MockFilterNodeBase>(node_options);
    // Enables the fake for delegation.
    mock_filter_node_base->DelegateToFake();
  }

  std::shared_ptr<MockFilterNodeBase> mock_filter_node_base;
};

/* \brief Create a dummy point cloud for publishing */
void create_dummy_cloud(sensor_msgs::msg::PointCloud2 & cloud)
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

// TODO(jilada): refactor in #1150
void check_pc(
  sensor_msgs::msg::PointCloud2 & msg1,
  sensor_msgs::msg::PointCloud2 & msg2)
{
  // Convert to using the wrapper
  const point_cloud_msg_wrapper::PointCloud2View<autoware::common::types::PointXYZI> msg1_wrapper{
    msg1};
  const point_cloud_msg_wrapper::PointCloud2View<autoware::common::types::PointXYZI> msg2_wrapper{
    msg2};

  EXPECT_EQ(msg1.header.frame_id, msg2.header.frame_id);
  EXPECT_EQ(msg1.header.stamp, msg2.header.stamp);

  EXPECT_EQ(msg1_wrapper.size(), msg2_wrapper.size());
  for (auto i = 0U; i < msg1_wrapper.size(); ++i) {
    EXPECT_EQ(msg1_wrapper[i], msg2_wrapper[i]);
  }
}

// Test using GMock
// cppcheck-suppress syntaxError
TEST_F(TestFilterNodeBase, TestParameters) {
  // Check that upon set up the parameters are set correctly
  EXPECT_THAT(mock_filter_node_base->test_param_1_, Eq(0.5));
  EXPECT_THAT(mock_filter_node_base->test_param_2_, Eq("frame_2"));

  // Set up parameter client
  auto client = std::make_shared<rclcpp::SyncParametersClient>(mock_filter_node_base);
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));
  const std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("max_queue_size", 10),
  };

  // Check that the get_node_parameters method in the MockFilterNodeBase class has been called
  // Since three parameters are changing the get_node_parameters will be called 3 times.
  EXPECT_CALL(*mock_filter_node_base, get_node_parameters(_)).Times(3);
  // Set new parameters
  client->set_parameters(parameters);
  rclcpp::spin_some(mock_filter_node_base);

  // Now change the child class specific parameters
  const std::vector<rclcpp::Parameter> child_class_parameters = {
    rclcpp::Parameter("test_param_1", 1.5),
    rclcpp::Parameter("test_param_2", "new_frame_2")
  };
  // Set new parameters
  client->set_parameters(child_class_parameters);
  rclcpp::spin_some(mock_filter_node_base);

  // Check that upon update the parameters are set correctly
  EXPECT_THAT(mock_filter_node_base->test_param_1_, Eq(1.5));
  EXPECT_THAT(mock_filter_node_base->test_param_2_, Eq("new_frame_2"));
}

// Test using the fake_test_node library
TEST_F(FilterNodeTest, TestFilter) {
  // Generate parameters
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("max_queue_size", 5);
  params.emplace_back("test_param_1", 0.5);
  params.emplace_back("test_param_2", "frame_2");

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  // Create instance of the TestFilter child class
  const auto node = std::make_shared<MockFilterNodeBase>(node_options);
  // Enables the fake for delegation.
  node->DelegateToFake();

  // Expect call to the filter method
  EXPECT_CALL(*node, filter).Times(AtLeast(1));

  // Set up message client and dummy point cloud
  sensor_msgs::msg::PointCloud2::SharedPtr last_received_msg{};
  sensor_msgs::msg::PointCloud2 msg;
  create_dummy_cloud(msg);
  auto fake_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
    "input", std::chrono::seconds{10LL}, rclcpp::SensorDataQoS().keep_last(10));
  auto result_cloud_subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
    "output", *node,
    [&last_received_msg](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      last_received_msg = msg;
    },
    std::chrono::seconds{10LL},
    rclcpp::SensorDataQoS().keep_last(10));

  const auto dt{std::chrono::milliseconds{100LL}};
  const auto max_wait_time{std::chrono::seconds{10LL}};
  auto time_passed{std::chrono::milliseconds{0LL}};
  while (!last_received_msg) {
    fake_cloud_pub->publish(msg);
    rclcpp::spin_some(node);
    rclcpp::spin_some(get_fake_node());
    std::this_thread::sleep_for(dt);
    time_passed += dt;
    if (time_passed > max_wait_time) {
      FAIL() << "Did not receive a message soon enough.";
    }
  }

  // Assert that the received message is the same
  check_pc(msg, *last_received_msg);
  SUCCEED();
}
}  // namespace
