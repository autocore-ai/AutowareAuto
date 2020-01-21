// Copyright 2019 Apex.AI, Inc.
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

#ifndef LOCALIZATION_NODES__LOCALIZATION_NODE_HPP_
#define LOCALIZATION_NODES__LOCALIZATION_NODE_HPP_

#include <localization_common/localizer_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/buffer_core.h>

namespace autoware
{
namespace localization
{
namespace localization_nodes
{

/// Base relative localizer node that publishes map->base_link relative
/// transform messages for a given observation source and map.
/// \tparam MsgT Message type to register against a map.
/// \tparam MapT Map type
/// \tparam LocalizerT Localizer type.
/// \tparam LocalizerConfigT Localizer configuration type.
template<typename MsgT, typename MapT, typename LocalizerT, typename LocalizerConfigT>
class RelativeLocalizerNode : rclcpp::Node
{
public:
  using LocalizerPtr = std::unique_ptr<localization_common::RelativeLocalizerBase<MsgT, MapT>>;

  RelativeLocalizerNode(const std::string & node_name, const std::string & name_space)
  : Node(node_name, name_space) {}

protected:
  void set_localizer(LocalizerPtr && localizer_ptr)
  {
    m_localizer = localizer_ptr;
  }

private:
  void core_callback()
  {
    // get msg
    // check and update maps
    // update_transforms()
    // init_geuss = m_localizer->compute_initial_guess(m_tf_buffer);
    // m_localizer->register_measurement(msg, init_guess);
    // if(m_localizer->validate_result()):
    // m_pub->publish(m_localizer->get_transformation());
  }

  // a tf2_listener can be used instead as well.
  void update_transforms();

  //
  void update_map(MapT &);

  // check the validity of current_map, warn/throw or request a new map
  bool validate_map();

  // tf buffer and map as well as pubs subs
  std::unique_ptr<localization_common::RelativeLocalizerBase<MsgT, MapT>> m_localizer;
  tf2::BufferCore m_tf_buffer;
  MapT m_map;
};
}          // namespace autoware
}      // namespace localization
}  // namespace localization_nodes
#endif
