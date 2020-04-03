// Copyright 2017-2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <iostream>
#include <iomanip>

using std::placeholders::_1;

#ifdef __cplusplus
extern "C"
{
#endif

static const rmw_qos_profile_t rmw_qos_unit_test_default =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  50,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

#ifdef __cplusplus
}
#endif

namespace rclcpp
{

class RCLCPP_PUBLIC UnitTestDefaultsQoS : public QoS
{
public:
  explicit
  UnitTestDefaultsQoS(
    const QoSInitialization & qos_initialization = (
      QoSInitialization::from_rmw(rmw_qos_unit_test_default)
  )) : QoS(qos_initialization, rmw_qos_unit_test_default)
  {}
};

}  // namespace rclcpp

class TestListener : public rclcpp::Node
{
public:
  /// \brief creates a node called test_listener
  TestListener()
  : Node("test_listener"),
    m_last_receive(this->now())
  {
    /// vv this version doesn't work for some reason?
    // auto fn1 = std::bind(&TestListener::sub_callback1, this, _1);
    auto fn2 =
      [this](const typename sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
      {
        const auto t_now = this->now();
        std::cout << "Got vlp, sz = " << msg->width << " time diff = " <<
        (t_now - msg->header.stamp).nanoseconds() / 1.0E6F <<
          " ms, Time since last message = " << (t_now - this->m_last_receive).nanoseconds() /
          1.0E6F << " ms\n";
        this->m_last_receive = t_now;
        std::flush(std::cout);
      };

    this->m_pointcloud_sub =
      create_subscription<sensor_msgs::msg::PointCloud2>("vlp16_cloud",
        rclcpp::UnitTestDefaultsQoS(), fn2);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_sub;
  rclcpp::Time m_last_receive;
};


// this file is simply a main file to create a ros1 style standalone node
int32_t main(int32_t argc, char * argv[])
{
  // init
  rclcpp::init(argc, argv);

  // setup executor for processing pub/sub callbacks
  rclcpp::executors::SingleThreadedExecutor exec;

  // executors only like shared_ptrs: dynamically allocated new listener node, fine because this
  //    is for testing
  auto vptr = std::make_shared<TestListener>();
  exec.add_node(vptr);

  // run endlessly
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
