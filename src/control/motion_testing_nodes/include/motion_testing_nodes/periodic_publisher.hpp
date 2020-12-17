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
#ifndef MOTION_TESTING_NODES__PERIODIC_PUBLISHER_HPP_
#define MOTION_TESTING_NODES__PERIODIC_PUBLISHER_HPP_

#include <motion_testing_nodes/visibility_control.hpp>
#include <motion_testing_nodes/wait_for_matched.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <list>
#include <string>

namespace motion
{
namespace motion_testing_nodes
{

struct MOTION_TESTING_NODES_PUBLIC PeriodCountTopic
{
  std::chrono::nanoseconds period;
  std::size_t count;
  std::string topic;
};

template<typename T>
class MOTION_TESTING_NODES_PUBLIC PeriodicPublisher
{
public:
  PeriodicPublisher(rclcpp::Node & node, const PeriodCountTopic & cfg, std::list<T> msgs = {})
  : m_msgs{msgs}
  {
    // Prune list if too long
    while (m_msgs.size() > cfg.count) {
      m_msgs.pop_back();
    }
    // Fill remainder of list
    for (auto idx = m_msgs.size(); idx < cfg.count; ++idx) {
      m_msgs.emplace_back(T{});
    }
    // Publisher
    m_pub =
      node.create_publisher<T>(cfg.topic, rclcpp::QoS{cfg.count}.transient_local().reliable());
    // Timer
    m_timer = node.create_wall_timer(
      cfg.period, [this]() -> void {
        if (!m_msgs.empty()) {
          const auto msg = m_msgs.front();
          m_pub->publish(msg);
          m_msgs.pop_front();
        }
      });
  }

  bool done() const noexcept {return m_msgs.empty();}

  void match(const std::size_t min_subscribers) const
  {
    (void)wait_for_matched(*m_pub, min_subscribers, std::chrono::seconds{3LL});
  }

private:
  std::list<T> m_msgs;
  typename rclcpp::Publisher<T>::SharedPtr m_pub{};
  rclcpp::TimerBase::SharedPtr m_timer{};
};  // class PeriodicPublisher

}  // namespace motion_testing_nodes
}  // namespace motion

#endif  // MOTION_TESTING_NODES__PERIODIC_PUBLISHER_HPP_
