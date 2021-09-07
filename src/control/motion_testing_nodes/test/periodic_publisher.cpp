// Copyright 2019 Christopher Ho
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
#include <gtest/gtest.h>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <motion_testing_nodes/periodic_publisher.hpp>

#include <memory>
#include <string>
#include <vector>

using motion::motion_testing_nodes::PeriodCountTopic;
using motion::motion_testing_nodes::PeriodicPublisher;

// Ugly global to persist some data across test cases
static int32_t count{0};

template<typename T>
class PeriodicPublisherTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
    ++count;
    std::string topic1{"periodic_publisher_test_topic1"};
    topic1 += std::to_string(count);
    cfg1_ = std::make_unique<PeriodCountTopic>(
      PeriodCountTopic{std::chrono::milliseconds{50LL}, 10U, topic1});
    std::string topic2{"periodic_publisher_test_topic2"};
    topic2 += std::to_string(count);
    cfg2_ = std::make_unique<PeriodCountTopic>(
      PeriodCountTopic{std::chrono::milliseconds{100LL}, 8U, topic2});
  }
  void TearDown()
  {
    (void)rclcpp::shutdown();
  }
  std::unique_ptr<PeriodCountTopic> cfg1_{};
  std::unique_ptr<PeriodCountTopic> cfg2_{};
};  // class PeriodicPublisherTest


using Command = autoware_auto_msgs::msg::VehicleControlCommand;
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TestTypes = ::testing::Types<State, Trajectory, Command>;

// cppcheck-suppress syntaxError
TYPED_TEST_CASE(PeriodicPublisherTest, TestTypes, );

TYPED_TEST(PeriodicPublisherTest, Basic)
{
  ASSERT_GT((this->cfg1_)->count, 0U);
  ASSERT_GT((this->cfg2_)->count, 0U);
  using T = TypeParam;
  using Pub = PeriodicPublisher<T>;
  std::string node_name{"periodic_publisher_test_node"};
  node_name += std::to_string(count);
  const auto node = std::make_shared<rclcpp::Node>(node_name);
  Pub pub1{*node, *(this->cfg1_)};
  Pub pub2{*node, *(this->cfg2_)};

  std::vector<T> v1, v2;
  v1.reserve((this->cfg1_)->count);
  v2.reserve((this->cfg2_)->count);
  std::string listener_name{"periodic_publisher_test_listener"};
  listener_name += std::to_string(count);
  const auto listener = std::make_shared<rclcpp::Node>(listener_name);
  const auto create_sub = [listener](const auto cfg, auto & v) -> auto {
      const auto qos = rclcpp::QoS{cfg.count}.reliable().transient_local();
      const auto fn = [&v](const typename T::SharedPtr msg) -> void {
          v.push_back(*msg);
        };
      return listener->create_subscription<T>(cfg.topic, qos, fn);
    };
  const auto sub1 = create_sub(*(this->cfg1_), v1);
  const auto sub2 = create_sub(*(this->cfg2_), v2);
  rclcpp::executors::SingleThreadedExecutor exec{};
  exec.add_node(node);
  exec.add_node(listener);

  while ((!pub1.done()) || (!pub2.done())) {
    exec.spin_some(std::chrono::milliseconds{100LL});
  }
  exec.spin_some(std::chrono::milliseconds{100LL});
  EXPECT_TRUE(pub1.done());
  EXPECT_TRUE(pub2.done());
  EXPECT_EQ(v1.size(), (this->cfg1_)->count);
  EXPECT_GE(v2.size() + 1U, (this->cfg2_)->count);
  EXPECT_GT(v2.size(), 0U);
  // TODO(c.ho) actually check period of arrival
}
