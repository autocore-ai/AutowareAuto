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
#ifndef MPC_CONTROLLER_NODE__MPC_CONTROLLER_NODE_HPP_
#define MPC_CONTROLLER_NODE__MPC_CONTROLLER_NODE_HPP_

#include <mpc_controller_node/visibility_control.hpp>
#include <controller_common_nodes/controller_base_node.hpp>
#include <mpc_controller/mpc_controller.hpp>

#include <string>

namespace motion
{
namespace control
{
namespace mpc_controller_node
{
class MPC_CONTROLLER_NODE_PUBLIC MpcControllerNode
  : public controller_common_nodes::ControllerBaseNode
{
public:
  /// Parameter file constructor
  MpcControllerNode(const std::string & name, const std::string & ns);
  /// Explicit constructor
  MpcControllerNode(
    const std::string & name,
    const std::string & ns,
    const std::string & command_topic,
    const std::string & state_topic,
    const std::string & tf_topic,
    const std::string & trajectory_topic,
    const std::string & diagnostic_topic,
    const std::string & static_tf_topic,
    const mpc_controller::Config & config);

private:
  rclcpp::Publisher<autoware_auto_msgs::msg::Trajectory>::SharedPtr m_debug_traj_pub{};
  rclcpp::TimerBase::SharedPtr m_debug_timer{};
};  // class MpcControllerNode
}  // namespace mpc_controller_node
}  // namespace control
}  // namespace motion

#endif  // MPC_CONTROLLER_NODE__MPC_CONTROLLER_NODE_HPP_
