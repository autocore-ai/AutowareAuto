// Copyright 2020 Apex.AI, Inc.
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#ifndef STATE_ESTIMATION_NODE__TIME_HPP_
#define STATE_ESTIMATION_NODE__TIME_HPP_


#include <common/types.hpp>

#include <chrono>
#include <utility>

namespace autoware
{
namespace prediction
{

///
/// @brief      This class describes a time reference frame. This is used to define in which clock a
///             particular time point lives.
///
enum class TimeReferenceFrame
{
  kGlobal,
  kMeasurement
};

///
/// @brief      This class describes a time point with respect to some time reference frame.
///
/// @tparam     kTimeReferenceFrame  A reference frame for the stored time_point.
///
template<TimeReferenceFrame kTimeReferenceFrame>
class Time : public std::chrono::system_clock::time_point
{
  using Base = std::chrono::system_clock::time_point;

public:
  /// @brief      Default contructor.
  Time() = default;
  ///
  /// @brief      Constructs a new instance from a time_point.
  ///
  /// @param[in]  time_point  A time point.
  ///
  explicit Time(const Base & time_point)
  : Base{time_point} {}
  ///
  /// @brief      Determines if the stored time_point is valid.
  ///
  /// @return     True if valid, False otherwise.
  ///
  common::types::bool8_t is_valid() const noexcept
  {
    return time_since_epoch() > Time::duration{0LL};
  }

  ///
  /// @brief      Add an operator + for two time points.
  ///
  /// @return     A new time point which is a sum of input ones.
  ///
  friend Time operator+(Time time_1, const Time & time_2)
  {
    return Time{time_1 + time_2.time_since_epoch()};
  }

  ///
  /// @brief      Overload operator +.
  ///
  ///             We cannot use the operator for the base class as we want to still return Time, not
  ///             an ordinary std::chrono::time_point.
  ///
  /// @param[in]  time       The base time point.
  /// @param[in]  duration   The duration to add to the time point.
  ///
  /// @tparam     IntT       Help integer type to pass into the duration template.
  /// @tparam     DurationT  Type of ratio used as a duration.
  ///
  /// @return     A new time point which represents the input time with duration added to it.
  ///
  /// @note       This must be templated as otherwise the compiler gets confused as to which
  ///             overload to pick in case the duration is not std::chrono::nanoseconds. If it does
  ///             not match then the duration parameter matches better with an overload from
  ///             std::chrono, while the first argument matches this class better. The compiler
  ///             cannot pick the best one and throws a warning. Thus we template this function to
  ///             make this choice ideal when used with Time class.
  ///
  template<typename IntT, typename DurationT>
  friend Time operator+(const Time & time, const std::chrono::duration<IntT, DurationT> & duration)
  {
    return Time{static_cast<const Base &>(time) + duration};
  }

  ///
  /// @brief      Overload operator -.
  ///
  ///             We cannot use the operator for the base class as we want to still return Time, not
  ///             an ordinary std::chrono::time_point.
  ///
  /// @param[in]  time       The base time point.
  /// @param[in]  duration   The duration to subtract from the time point.
  ///
  /// @tparam     IntT       Help integer type to pass into the duration template.
  /// @tparam     DurationT  Type of ratio used as a duration.
  ///
  /// @return     A new time point which represents the input time with duration subtracted from it.
  ///
  /// @note       This must be templated as otherwise the compiler gets confused as to which
  ///             overload to pick in case the duration is not std::chrono::nanoseconds. If it does
  ///             not match then the duration parameter matches better with an overload from
  ///             std::chrono, while the first argument matches this class better. The compiler
  ///             cannot pick the best one and throws a warning. Thus we template this function to
  ///             make this choice ideal when used with Time class.
  ///
  template<typename IntT, typename DurationT>
  friend Time operator-(const Time & time, const std::chrono::duration<IntT, DurationT> & duration)
  {
    return Time{static_cast<const Base &>(time) - duration};
  }
};

/// @brief Utility typedef to define time based on a measurement clock.
using MeasurementBasedTime = Time<TimeReferenceFrame::kMeasurement>;
/// @brief Utility typedef to define time based on some global clock.
using GlobalTime = Time<TimeReferenceFrame::kGlobal>;


}  // namespace prediction
}  // namespace autoware

#endif  // STATE_ESTIMATION_NODE__TIME_HPP_
