// Copyright 2020 the Autoware Foundation
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

#ifndef POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
#define POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_

#include <point_cloud_mapping/visibility_control.hpp>
#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_mapping/map.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <lidar_utils/point_cloud_utils.hpp>
#include <point_cloud_mapping/policies.hpp>
#include <helper_functions/message_adapters.hpp>
#include <time_utils/time_utils.hpp>
#include <experimental/optional>
#include <memory>
#include <string>
#include <utility>
#include <type_traits>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{
template<typename MapIncrementT>
using RegistrationResult = std::pair<std::reference_wrapper<const MapIncrementT>,
    geometry_msgs::msg::PoseWithCovarianceStamped>;

/// Base summary class for mapper registration
/// \tparam LocalizerSummaryT Registration summary type of the mapper's localizer.
/// \tparam MapIncrementT Map increment type used in the mapper.
template<typename LocalizerSummaryT, typename MapIncrementT>
struct POINT_CLOUD_MAPPING_PUBLIC MapperRegistrationSummaryBase
{
public:
  using MaybeLocalizerSummary = std::experimental::optional<LocalizerSummaryT>;
//  /// Constructor
//  /// \param increment const ref. to the Increment from the latest registration.
//  /// \param update_summary Map update summary from the latest map insertion.
//  /// \param localization_summary Localizer registration summary.
//  MapperRegistrationSummaryBase(
//    MapIncrementT increment,
//    MapUpdateSummary update_summary, MaybeLocalizerSummary localization_summary)
//  : map_increment{increment}, map_update_summary{update_summary},
//    localizer_summary{localization_summary} {}

  MapIncrementT map_increment;
  MapUpdateSummary map_update_summary;
  MaybeLocalizerSummary localizer_summary;
};

using common::helper_functions::message_field_adapters::get_frame_id;
using common::helper_functions::message_field_adapters::get_stamp;

/// Virtual base class of the mapper. The mapper is expected to:
/// * Receive observations
/// * Register observations to the existing map
/// * Compute and pass the map increment to the used map representation and
/// export the map according to the provided policies.
/// * Fulfill the API of a RelativeLocalizer.
/// \tparam LocalizerT Localizer to be used for registration.
/// \tparam MapRepresentationT  Map representation type used in the mapper.
/// \tparam ObservationMsgT Type of observation used to build the map.
/// \tparam MapIncrementT Type of increment to extend the map representation.
/// \tparam RegistrationSummaryT Summary class to report the result of mapper registration.
/// \tparam WriteTriggerPolicyT Policy specifying in what condition to write to a file.
/// \tparam PrefixGeneratorT Functor that generates filename prefixes for a given base prefix.
template<typename LocalizerT, typename MapRepresentationT,
  typename ObservationMsgT, typename MapIncrementT, typename RegistrationSummaryT,
  class WriteTriggerPolicyT, class PrefixGeneratorT>
class POINT_CLOUD_MAPPING_PUBLIC MapperBase
  : public localization::localization_common::RelativeLocalizerBase<
    ObservationMsgT, MapIncrementT, RegistrationSummaryT>
{
public:
  static_assert(std::is_same<MapIncrementT, typename LocalizerT::MapMsg>::value,
    "MapperBase and the utilized Relative Localizer must use the same map increment type");
  static_assert(std::is_same<ObservationMsgT, typename LocalizerT::InputMsg>::value,
    "MapperBase and the utilized Relative Localizer must use the same observation type");

  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  using MaybeLocalizerSummary =
    std::experimental::optional<typename LocalizerT::RegistrationSummary>;
  using MapIncrementPtrT = std::shared_ptr<MapIncrementT>;
  using ConstMapIncrementPtrT = std::shared_ptr<const MapIncrementT>;
  using RegistrationSummary = RegistrationSummaryT;
  using LocalizerBase = localization::localization_common::
    RelativeLocalizerBase<ObservationMsgT, MapIncrementT, typename LocalizerT::RegistrationSummary>;
  using LocalizerBasePtr = std::unique_ptr<LocalizerBase>;

  /// Constructor.
  /// \param map_filename_prefix Base filename prefix that will be used to
  /// generate the file name.
  /// \param localizer_ptr Rvalue reference to the localizer pointer.
  /// \param map_frame_id  Frame ID of the map.
  /// \param map Rvalue reference to the map representation pointer.
  MapperBase(
    const std::string & map_filename_prefix,
    MapRepresentationT && map,
    LocalizerBasePtr && localizer_ptr,
    const std::string map_frame_id)
  : m_base_fn_prefix{map_filename_prefix},
    m_map{std::forward<MapRepresentationT>(map)},
    m_localizer_ptr{std::forward<LocalizerBasePtr>(localizer_ptr)},
    m_frame_id{map_frame_id}
  {
    this->set_map_valid();
  }

  /// Insert an observation to the mapper. This observation gets registered to the existing map
  /// of the localizer. The increment is computed using the registration result and the observation
  /// and finally the increment is passed to the map representation.
  RegistrationSummaryT  register_measurement_impl(
    const ObservationMsgT & msg, const TransformStamped & transform_initial,
    PoseWithCovarianceStamped & pose_out) override
  {
    PoseWithCovarianceStamped registered_pose;

    MaybeLocalizerSummary localization_summary{std::experimental::nullopt};

    if (m_map.size() > 0U) {
      if (!m_localizer_ptr->map_valid()) {
        throw std::runtime_error("MapperBase: Map representation has data but "
                "the localizer's map is at an invalid state.");
      }
      localization_summary =
        m_localizer_ptr->register_measurement(msg, transform_initial, pose_out);
    } else {
      // If the map is empty, there's nothing to register to. We can place the cloud at the
      // center of the map.
      pose_out = get_identity(get_stamp(msg), m_frame_id);
    }

    const auto & increment_ptr = get_map_increment(msg, pose_out);

    const auto & update_summary = update_map(*increment_ptr, registered_pose);

    if (m_trigger_policy.ready(m_map)) {
      write_to_file();
    }

    return make_summary(increment_ptr, update_summary, localization_summary);
  }

  const std::string & map_frame_id() const noexcept override
  {
    return m_frame_id;
  }

  void set_map_impl(const MapIncrementT & msg) override
  {
    m_map.clear();
    insert_to_map_impl(msg);
  }

  std::chrono::system_clock::time_point map_stamp() const noexcept override
  {
    return m_current_stamp;
  }

  void insert_to_map_impl(const MapIncrementT & msg) override
  {
    const auto & msg_frame = get_frame_id(msg);
    if (msg_frame != m_frame_id) {
      throw std::runtime_error("MapperBase: Can't insert a map that is in a different frame.");
    }
    update_map(msg, get_identity(get_stamp(msg), m_frame_id));
  }

  /// Virtual base destructor. Triggers map writing in case there is unwritten data in the map.
  virtual ~MapperBase()
  {
    write_to_file();
  }

protected:
  void write_to_file() const
  {
    if (m_map.size() > 0U) {
      m_map.write(m_fn_prefix_generator.get(m_base_fn_prefix));
    }
  }

  void set_write_trigger(WriteTriggerPolicyT && write_trigger)
  {
    m_trigger_policy = std::move(write_trigger);
  }

  void set_prefix_generator(PrefixGeneratorT && prefix_generator)
  {
    m_fn_prefix_generator = std::move(prefix_generator);
  }

  PoseWithCovarianceStamped get_identity(
    builtin_interfaces::msg::Time stamp,
    const std::string frame_id) const noexcept
  {
    PoseWithCovarianceStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id;
    out.pose.pose.orientation.w = 1.0;
    return out;
  }

private:
  virtual RegistrationSummaryT make_summary(
    const ConstMapIncrementPtrT & increment,
    const MapUpdateSummary & map_update_summary,
    const MaybeLocalizerSummary & localization_summary) const noexcept = 0;

  /// Pass the increment to the map. The default behavior is to expect that the map
  /// and the localizer have independent representations and push the increment to both
  /// the underlying map representation and the localizer.
  /// \param increment Increment to pass to the maps.
  /// \param pose Pose to be inserted with the increment.
  auto update_map(
    const MapIncrementT & increment,
    PoseWithCovarianceStamped pose)
  {
    const auto insert_summary = m_map.try_add_observation(increment, pose);

    // Let's update the stamp at the start of each map in case someone needs it.
    if (insert_summary.update_type == MapUpdateType::NEW) {
      m_current_stamp = time_utils::from_message(pose.header.stamp);
    }
    // Only also insert to the localizer's map if the maps are not shared.
    // TODO(yunus.caliskan): switch to `if constexpr` when c++17 is available.
    if (m_map.storage_mode() == MapStorageMode::Independent) {
      switch (insert_summary.update_type) {
        case MapUpdateType::NEW:
          m_localizer_ptr->set_map(increment);
          break;
        case MapUpdateType::UPDATE:
          m_localizer_ptr->insert_to_map(increment);
          break;
        case MapUpdateType::NO_CHANGE:
        default:
          break;
      }
    }
    return insert_summary;
  }

  /// Concert the observation to a map increment ready to be inserted to the map.
  /// \param observation Observation to be used for computing the increment.
  /// \param registered_pose Registered pose of the observation.
  /// \return Pointer to the map increment that is desired by the map representation.
  virtual ConstMapIncrementPtrT get_map_increment(
    const ObservationMsgT & observation,
    const PoseWithCovarianceStamped & registered_pose) = 0;

  std::string m_base_fn_prefix;
  MapRepresentationT m_map;
  LocalizerBasePtr m_localizer_ptr;
  WriteTriggerPolicyT m_trigger_policy;
  PrefixGeneratorT m_fn_prefix_generator;
  std::string m_frame_id;
  std::chrono::system_clock::time_point m_current_stamp{
    std::chrono::system_clock::time_point::min()};
};

using Cloud = sensor_msgs::msg::PointCloud2;
using CloudPtr = std::shared_ptr<Cloud>;
using ConstCloudPtr = std::shared_ptr<const Cloud>;

/// PointCloudMapper creates maps from point cloud observations.
/// \tparam LocalizerT Localizer type used for registration.
/// \tparam WriteTriggerPolicyT Policy specifying in what condition to write to a file.
/// \tparam PrefixGeneratorT Functor that generates filename prefixes for a given base prefix.
template<typename LocalizerT, typename MapRepresentationT,
  class WriteTriggerPolicyT, class FileNamePrefixGeneratorT>
class PointCloudMapper : public MapperBase<LocalizerT, MapRepresentationT,
    sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
    MapperRegistrationSummaryBase<typename LocalizerT::RegistrationSummary,
    ConstCloudPtr>,
    WriteTriggerPolicyT, FileNamePrefixGeneratorT>
{
public:
  using RegistrationSummary =
    MapperRegistrationSummaryBase<typename LocalizerT::RegistrationSummary,
      ConstCloudPtr>;
  using Base = MapperBase<LocalizerT, MapRepresentationT,
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2,
      RegistrationSummary, WriteTriggerPolicyT, FileNamePrefixGeneratorT>;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using LocalizerBasePtr = typename Base::LocalizerBasePtr;

  /// Constructor.
  /// \param map_filename_prefix Base filename prefix that will be used to
  /// generate the file name.
  /// \param localizer_ptr Rvalue reference to the localizer pointer.
  /// \param map Rvalue reference to the map representation pointer.
  /// \param map_frame Map frame id.
  PointCloudMapper(
    const std::string & map_filename_prefix,
    MapRepresentationT && map, LocalizerBasePtr && localizer_ptr,
    const std::string & map_frame)
  : Base(map_filename_prefix, std::forward<MapRepresentationT>(map),
      std::forward<LocalizerBasePtr>(localizer_ptr), map_frame),
    m_cached_increment_ptr{std::make_shared<Cloud>()}
  {
    common::lidar_utils::init_pcl_msg(*m_cached_increment_ptr, map_frame);
  }

private:
  /// Transform the observed point cloud into the map frame using the registered
  /// pose.
  /// \param observation Point cloud observation.
  /// \param registered_pose Registered pose of the observation.
  /// \return Pointer to the transformed point cloud;
  ConstCloudPtr get_map_increment(
    const Cloud & observation,
    const PoseWithCovarianceStamped & registered_pose) override
  {
    reset_cached_msg(get_msg_size(observation));
    // Convert pose to transform for `doTransform()`
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = registered_pose.header.stamp;
    tf.header.frame_id = registered_pose.header.frame_id;
    tf.child_frame_id = observation.header.frame_id;
    const auto & trans = registered_pose.pose.pose.position;
    const auto & rot = registered_pose.pose.pose.orientation;
    tf.transform.translation.set__x(trans.x).set__y(trans.y).set__z(trans.z);
    tf.transform.rotation.set__x(rot.x).set__y(rot.y).set__z(rot.z).set__w(rot.w);

    tf2::doTransform(observation, *m_cached_increment_ptr, tf);
    return m_cached_increment_ptr;
  }

private:
  void reset_cached_msg(std::size_t size)
  {
    sensor_msgs::PointCloud2Modifier inc_modifier{*m_cached_increment_ptr};
    inc_modifier.clear();
    inc_modifier.resize(size);
  }
  std::size_t get_msg_size(const Cloud & msg) const
  {
    const auto safe_indices = common::lidar_utils::sanitize_point_cloud(msg);
    // Only do the division when necessary.
    return (safe_indices.data_length == msg.data.size()) ?
           msg.width : (safe_indices.data_length / safe_indices.point_step);
  }

  RegistrationSummary make_summary(
    const ConstCloudPtr & increment,
    const MapUpdateSummary & map_update_summary,
    const typename Base::MaybeLocalizerSummary & localization_summary) const noexcept override
  {
    return RegistrationSummary{increment, map_update_summary, localization_summary};
  }

  CloudPtr m_cached_increment_ptr;
};

}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__POINT_CLOUD_MAPPER_HPP_
