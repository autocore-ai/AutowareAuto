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

#ifndef POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
#define POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_

#include <point_cloud_mapping/visibility_control.hpp>
#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_mapping/map.hpp>
#include <memory>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{
template<typename LocalizerT, typename MapRepresentationT, typename MapMsgT,
  typename ObservationMsgT>
class POINT_CLOUD_MAPPING_PUBLIC MapperBase
{
public:
  using LocalizerBase = localization::localization_common::RelativeLocalizerBase<ObservationMsgT,
      MapMsgT,
      typename LocalizerT::RegistrationSummary>;
  using PoseWithCovarianceStamped = typename LocalizerBase::PoseWithCovarianceStamped;
  using TransformStamped = typename LocalizerBase::Transform;

  void observe(const ObservationMsgT & observation, const TransformStamped & transform_initial)
  {
    PoseWithCovarianceStamped registered_pose;
    m_localizer->register_measurement(observation, transform_initial, registered_pose);
    const auto & increment = get_map_increment(observation, registered_pose);
    const auto insert_summary = try_add_observation(increment, registered_pose);
    switch (insert_summary.update_type) {
      case MapUpdateType::NEW:
        m_localizer->set_map(m_map->export_map());
        break;
      case MapUpdateType::UPDATE:
        m_localizer->insert_to_map(get_map_increment(observation, registered_pose));
        break;
      case MapUpdateType::NO_CHANGE:
      default:
        break;
    }
    check_to_write(*m_map);
  }

private:
  virtual const MapMsgT & get_map_increment(
    const ObservationMsgT & input_msg,
    const PoseWithCovarianceStamped & registered_pose) = 0;

  virtual void check_to_write(const MapRepresentationT & map) = 0;

  std::unique_ptr<LocalizerBase> m_localizer;
  std::unique_ptr<MapRepresentationBase<MapMsgT>> m_map;
};
}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
