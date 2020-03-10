// Copyright 2020 Apex.AI, Inc.
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

#include <gtest/gtest.h>
#include <localization_common/localizer_base.hpp>

namespace autoware
{
namespace localization
{
namespace localization_common
{
constexpr int ERR_CODE = -99;
constexpr int INVALID_ID = -1;

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Transform = geometry_msgs::msg::TransformStamped;

std::string merge_ids(int pose, int init, int map)
{
  return std::to_string(pose) + "_" + std::to_string(init) + std::to_string(map);
}

template<typename MsgT>
std::string get_id(const MsgT & msg)
{
  return msg.header.frame_id;
}
template<typename MsgT>
void set_id(MsgT & msg, const std::string & id)
{
  msg.header.frame_id = id;
}

class TestLocalizer : public RelativeLocalizerBase<int, int>
{
public:
  PoseWithCovarianceStamped register_measurement_impl(
    const int & msg, const Transform & transform_initial) override
  {
    PoseWithCovarianceStamped pose{};
    set_id(pose, merge_ids(msg, std::stoi(get_id(transform_initial)), m_map_id));
    return pose;
  }
  const std::string & map_frame_id() const noexcept override
  {
    return m_map_frame;
  }

  /// Get the timestamp of the current map.
  std::chrono::system_clock::time_point map_stamp() const noexcept override
  {
    std::chrono::system_clock::time_point::min();
  }

  void set_map_impl(const int & map) override
  {
    if (map == ERR_CODE) {
      throw std::runtime_error("");
    }
    m_map_id = map;
    m_map_frame = std::to_string(m_map_id);
  }

  int m_map_id{INVALID_ID};
  std::string m_map_frame{""};
};


}          // namespace autoware
}      // namespace localization
}  // namespace localization_common
