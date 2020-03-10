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

#ifndef LOCALIZATION_COMMON__LOCALIZER_BASE_HPP_
#define LOCALIZATION_COMMON__LOCALIZER_BASE_HPP_

#include <localization_common/visibility_control.hpp>
#include <tf2/buffer_core.h>
#include <geometry_msgs/msg/transform.hpp>
#include <localization_common/initialization.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <string>

namespace autoware
{
namespace localization
{
namespace localization_common
{
using Real = double;
/// The base class for relative localizers.
/// \tparam InputMsgT Message type that will be registered against a map.
/// \tparam MapMsgT Map type.
template<typename InputMsgT, typename MapMsgT>
class LOCALIZATION_COMMON_PUBLIC RelativeLocalizerBase
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Transform = geometry_msgs::msg::TransformStamped;

  /// Registers a measurement to the current map and returns the
  /// estimated pose of the vehicle and its validity.
  /// \param[in] msg Measurement message to register.
  /// \param[in] transform_initial Initial guess of the pose to initialize the localizer with
  /// in iterative processes like solving optimization problems.
  /// \return Resulting pose estimate after registration.
  /// \throws If the result is invalid and cannot be used. Defined in the implementation.
  PoseWithCovarianceStamped register_measurement(
    const InputMsgT & msg, const Transform & transform_initial)
  {
    if (!m_map_valid) {
      on_register_without_map();
    }
    return register_measurement_impl(msg, transform_initial);
  }

  /// Set map.
  /// \param msg Map message.
  void set_map(const MapMsgT & msg)
  {
    try {
      set_map_impl(msg);
      set_map_valid();
    } catch (...) {
      set_map_invalid();
      on_bad_map(std::current_exception());
    }
  }

  /// Get the frame id of the current map.
  virtual const std::string & map_frame_id() const noexcept = 0;

  /// Get the timestamp of the current map.
  virtual std::chrono::system_clock::time_point map_stamp() const noexcept = 0;

  bool map_valid() const noexcept
  {
    return m_map_valid;
  }

  virtual ~RelativeLocalizerBase() = default;

protected:
  /// `set_map` implementation.
  virtual void set_map_impl(const MapMsgT & msg) = 0;

  /// Action to take on failure to set a new map.
  virtual void on_bad_map(std::exception_ptr eptr)
  {
    if (eptr) {
      std::rethrow_exception(eptr);
    }
  }

  /// `register_measurement(...)` implementation.
  virtual PoseWithCovarianceStamped register_measurement_impl(
    const InputMsgT & msg, const Transform & transform_initial) = 0;

  /// Set current map as valid.
  void set_map_valid() noexcept
  {
    m_map_valid = true;
  }

  /// Set current map as invalid.
  void set_map_invalid() noexcept
  {
    m_map_valid = false;
  }

  /// Action to take when a measurement is attempted to be set without a valid map.
  virtual void on_register_without_map()
  {
    throw std::logic_error("RelativeLocalizerBase: Cannot register a measurement without a valid"
            " map. Either no valid map is set yet or an invalid map had been "
            "attempted to be set.");
  }

private:
  bool m_map_valid{false};
};
}  // namespace localization_common
}  // namespace localization
}  // namespace autoware

#endif  // LOCALIZATION_COMMON__LOCALIZER_BASE_HPP_
