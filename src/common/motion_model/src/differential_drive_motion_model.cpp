// Copyright 2021 Apex.AI, Inc.
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
// Developed by Apex.AI, Inc.

#include <motion_model/differential_drive_motion_model.hpp>

#include <common/types.hpp>
#include <helper_functions/float_comparisons.hpp>

#include <cmath>

namespace
{
constexpr auto kEpsilon = 0.00001F;

using std::cos;
using std::sin;

using autoware::common::state_vector::variable::X;
using autoware::common::state_vector::variable::Y;
using autoware::common::state_vector::variable::YAW;
using autoware::common::state_vector::variable::YAW_CHANGE_RATE;
using autoware::common::state_vector::variable::YAW_CHANGE_ACCELERATION;
using autoware::common::state_vector::variable::XY_VELOCITY;
using autoware::common::state_vector::variable::XY_ACCELERATION;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

}  // namespace

namespace autoware
{
namespace common
{
namespace motion_model
{

template<typename ScalarT>
typename CvtrMotionModel<ScalarT>::State cvtr_predict(
  const typename CvtrMotionModel<ScalarT>::State & state,
  const std::chrono::nanoseconds & dt)
{
  typename CvtrMotionModel<ScalarT>::State new_state{};
  const auto t = std::chrono::duration<ScalarT>{dt}.count();
  const auto theta = state.template at<YAW>();
  const auto x = state.template at<X>();
  const auto y = state.template at<Y>();
  const auto v = state.template at<XY_VELOCITY>();
  const auto w = state.template at<YAW_CHANGE_RATE>();
  if (common::helper_functions::comparisons::abs_eq_zero(w, static_cast<ScalarT>(kEpsilon))) {
    new_state.template at<X>() = x + t * v * cos(theta);
    new_state.template at<Y>() = y + t * v * sin(theta);
  } else {
    new_state.template at<X>() = x + v * (-sin(theta) + sin(t * w + theta)) / w;
    new_state.template at<Y>() = y + v * (cos(theta) - cos(t * w + theta)) / w;
  }
  new_state.template at<XY_VELOCITY>() = v;
  new_state.template at<YAW>() = t * w + theta;
  new_state.template at<YAW_CHANGE_RATE>() = w;
  return new_state;
}

template<typename ScalarT>
typename CvtrMotionModel<ScalarT>::State::Matrix cvtr_jacobian(
  const typename CvtrMotionModel<ScalarT>::State & state,
  const std::chrono::nanoseconds & dt)
{
  using State = typename CvtrMotionModel<ScalarT>::State;
  typename State::Matrix jacobian{State::Matrix::Identity()};
  const auto t = std::chrono::duration<ScalarT>{dt}.count();
  const auto theta = state.template at<YAW>();
  const auto v = state.template at<XY_VELOCITY>();
  const auto w = state.template at<YAW_CHANGE_RATE>();
  const auto x_index = State::template index_of<X>();
  const auto y_index = State::template index_of<Y>();
  const auto v_index = State::template index_of<XY_VELOCITY>();
  const auto theta_index = State::template index_of<YAW>();
  const auto w_index = State::template index_of<YAW_CHANGE_RATE>();
  if (common::helper_functions::comparisons::abs_eq_zero(w, static_cast<ScalarT>(kEpsilon))) {
    const auto sin_theta = sin(theta);
    const auto cos_theta = cos(theta);
    const auto t_sin_theta = t * sin_theta;
    const auto t_v_sin_theta = v * t_sin_theta;
    const auto t_cos_theta = t * cos_theta;
    const auto t_v_cos_theta = v * t_cos_theta;
    // X row non-identity entries
    jacobian(x_index, theta_index) = -t_v_sin_theta;
    jacobian(x_index, v_index) = t_cos_theta;
    jacobian(x_index, w_index) = -t * t_v_sin_theta;
    // Y row non-identity entries
    jacobian(y_index, theta_index) = t_v_cos_theta;
    jacobian(y_index, v_index) = t_sin_theta;
    jacobian(y_index, w_index) = t * t_v_cos_theta;
  } else {
    const auto inv_w = static_cast<ScalarT>(1.0) / w;
    const auto next_theta = theta + t * w;
    const auto sin_next_theta = sin(next_theta);
    const auto cos_next_theta = cos(next_theta);
    const auto sin_theta_diff = sin_next_theta - sin(theta);
    const auto sin_theta_diff_inv_w = inv_w * sin_theta_diff;
    const auto v_sin_theta_diff_inv_w = v * sin_theta_diff_inv_w;
    const auto cos_theta_diff = cos_next_theta - cos(theta);
    const auto cos_theta_diff_inv_w = inv_w * cos_theta_diff;
    const auto v_cos_theta_diff_inv_w = v * cos_theta_diff_inv_w;
    // X row non-identity entries
    jacobian(x_index, theta_index) = v_cos_theta_diff_inv_w;
    jacobian(x_index, v_index) = sin_theta_diff_inv_w;
    jacobian(x_index, w_index) = t * v * inv_w * cos_next_theta - inv_w * v_sin_theta_diff_inv_w;
    // Y row non-identity entries
    jacobian(y_index, theta_index) = v_sin_theta_diff_inv_w;
    jacobian(y_index, v_index) = -cos_theta_diff_inv_w;
    jacobian(y_index, w_index) = t * v * inv_w * sin_next_theta + inv_w * v_cos_theta_diff_inv_w;
    // Velocity row non-identity entries
    jacobian(v_index, w_index) = t;
  }
  return jacobian;
}


template<typename ScalarT>
typename CatrMotionModel<ScalarT>::State catr_predict(
  const typename CatrMotionModel<ScalarT>::State & state,
  const std::chrono::nanoseconds & dt)
{
  typename CatrMotionModel<ScalarT>::State new_state{};
  const auto t = std::chrono::duration<ScalarT>{dt}.count();
  const auto theta = state.template at<YAW>();
  const auto x = state.template at<X>();
  const auto y = state.template at<Y>();
  const auto v = state.template at<XY_VELOCITY>();
  const auto a = state.template at<XY_ACCELERATION>();
  const auto w = state.template at<YAW_CHANGE_RATE>();
  const auto half_t_squared_a = static_cast<ScalarT>(0.5) * t * t * a;
  if (common::helper_functions::comparisons::abs_eq_zero(w, static_cast<ScalarT>(kEpsilon))) {
    new_state.template at<X>() = x + half_t_squared_a * cos(theta) + t * v * cos(theta);
    new_state.template at<Y>() = y + half_t_squared_a * sin(theta) + t * v * sin(theta);
  } else {
    const auto t_w_plus_theta = t * w + theta;
    const auto inv_w = static_cast<ScalarT>(1.0) / w;
    const auto inv_w_2 = inv_w * inv_w;
    new_state.template at<X>() = x +
      inv_w * t * a * sin(t_w_plus_theta) -
      inv_w * v * sin(theta) +
      inv_w * v * sin(t_w_plus_theta) +
      inv_w_2 * a * (-cos(theta) + cos(t_w_plus_theta));
    new_state.template at<Y>() = y -
      inv_w * t * a * cos(t_w_plus_theta) +
      inv_w * v * cos(theta) -
      inv_w * v * cos(t_w_plus_theta) +
      inv_w_2 * a * (-sin(theta) + sin(t_w_plus_theta));
  }
  new_state.template at<YAW>() = theta + w * t;
  new_state.template at<YAW_CHANGE_RATE>() = w;
  new_state.template at<XY_VELOCITY>() = v + t * a;
  new_state.template at<XY_ACCELERATION>() = a;
  return new_state;
}

template<typename ScalarT>
typename CatrMotionModel<ScalarT>::State::Matrix catr_jacobian(
  const typename CatrMotionModel<ScalarT>::State & state,
  const std::chrono::nanoseconds & dt)
{
  using State = typename CatrMotionModel<ScalarT>::State;
  typename State::Matrix jacobian{State::Matrix::Identity()};
  const auto t = std::chrono::duration<ScalarT>{dt}.count();
  const auto theta = state.template at<YAW>();
  const auto v = state.template at<XY_VELOCITY>();
  const auto a = state.template at<XY_ACCELERATION>();
  const auto w = state.template at<YAW_CHANGE_RATE>();
  const auto x_index = State::template index_of<X>();
  const auto y_index = State::template index_of<Y>();
  const auto theta_index = State::template index_of<YAW>();
  const auto v_index = State::template index_of<XY_VELOCITY>();
  const auto w_index = State::template index_of<YAW_CHANGE_RATE>();
  const auto a_index = State::template index_of<XY_ACCELERATION>();
  if (common::helper_functions::comparisons::abs_eq_zero(w, static_cast<ScalarT>(kEpsilon))) {
    const auto half = static_cast<ScalarT>(0.5);
    // X row non-identity entries
    jacobian(x_index, theta_index) = -half * t * t * a * sin(theta) - t * v * sin(theta);
    jacobian(x_index, v_index) = t * cos(theta);
    jacobian(x_index, w_index) = -half * t * t * t * a * sin(theta) - t * t * v * sin(theta);
    jacobian(x_index, a_index) = half * t * t * cos(theta);
    // Y row non-identity entries
    jacobian(y_index, theta_index) = half * t * t * a * cos(theta) + t * v * cos(theta);
    jacobian(y_index, v_index) = t * sin(theta);
    jacobian(y_index, w_index) = half * t * t * t * a * cos(theta) + t * t * v * cos(theta);
    jacobian(y_index, a_index) = half * t * t * sin(theta);
    // Velocity row non-identity entries
    jacobian(v_index, a_index) = t;
  } else {
    const auto inv_w = static_cast<ScalarT>(1.0) / w;
    const auto next_theta = theta + t * w;
    const auto sin_next_theta = sin(next_theta);
    const auto cos_next_theta = cos(next_theta);
    const auto sin_theta_diff = sin_next_theta - sin(theta);
    const auto sin_theta_diff_inv_w = inv_w * sin_theta_diff;
    const auto v_sin_theta_diff_inv_w = v * sin_theta_diff_inv_w;
    const auto cos_theta_diff = cos_next_theta - cos(theta);
    const auto cos_theta_diff_inv_w = inv_w * cos_theta_diff;
    const auto v_cos_theta_diff_inv_w = v * cos_theta_diff_inv_w;
    const auto two = static_cast<ScalarT>(2.0);
    // X row non-identity entries
    jacobian(x_index, theta_index) =
      t * a * inv_w * cos_next_theta + v_cos_theta_diff_inv_w - a * inv_w * sin_theta_diff_inv_w;
    jacobian(x_index, v_index) = sin_theta_diff_inv_w;
    jacobian(x_index, w_index) =
      t * t * a * inv_w * cos_next_theta +
      t * v * inv_w * cos_next_theta -
      two * t * a * inv_w * inv_w * sin_next_theta -
      inv_w * v_sin_theta_diff_inv_w -
      two * a * inv_w * inv_w * cos_theta_diff_inv_w;
    jacobian(x_index, a_index) = t * inv_w * sin_next_theta + inv_w * cos_theta_diff_inv_w;
    // Y row non-identity entries.
    jacobian(y_index, theta_index) =
      t * a * inv_w * sin_next_theta + v_sin_theta_diff_inv_w + a * inv_w * cos_theta_diff_inv_w;
    jacobian(y_index, v_index) = -cos_theta_diff_inv_w;
    jacobian(y_index, w_index) =
      t * t * a * inv_w * sin_next_theta +
      t * v * inv_w * sin_next_theta +
      two * t * a * inv_w * inv_w * cos_next_theta +
      inv_w * v_cos_theta_diff_inv_w -
      two * a * inv_w * inv_w * sin_theta_diff_inv_w;
    jacobian(y_index, a_index) = -t * inv_w * cos_next_theta + inv_w * sin_theta_diff_inv_w;
    // Velocity row non-identity entries.
    jacobian(v_index, a_index) = t;
    // Theta changes linearly with turn rate.
    jacobian(theta_index, w_index) = t;
  }
  return jacobian;
}

template<>
MOTION_MODEL_PUBLIC CvtrMotionModel32::State CvtrMotionModel32::crtp_predict(
  const CvtrMotionModel32::State & state,
  const std::chrono::nanoseconds & dt) const
{
  return cvtr_predict<float32_t>(state, dt);
}

template<>
MOTION_MODEL_PUBLIC CvtrMotionModel64::State CvtrMotionModel64::crtp_predict(
  const CvtrMotionModel64::State & state,
  const std::chrono::nanoseconds & dt) const
{
  return cvtr_predict<float64_t>(state, dt);
}

template<>
MOTION_MODEL_PUBLIC CvtrMotionModel32::State::Matrix CvtrMotionModel32::crtp_jacobian(
  const CvtrMotionModel32::State & state,
  const std::chrono::nanoseconds & dt) const
{
  return cvtr_jacobian<float32_t>(state, dt);
}

template<>
MOTION_MODEL_PUBLIC CvtrMotionModel64::State::Matrix CvtrMotionModel64::crtp_jacobian(
  const CvtrMotionModel64::State & state,
  const std::chrono::nanoseconds & dt) const
{
  return cvtr_jacobian<float64_t>(state, dt);
}

template<>
MOTION_MODEL_PUBLIC CatrMotionModel32::State CatrMotionModel32::crtp_predict(
  const CatrMotionModel32::State & state,
  const std::chrono::nanoseconds & dt) const
{
  return catr_predict<float32_t>(state, dt);
}

template<>
MOTION_MODEL_PUBLIC CatrMotionModel64::State CatrMotionModel64::crtp_predict(
  const CatrMotionModel64::State & state,
  const std::chrono::nanoseconds & dt) const
{
  return catr_predict<float64_t>(state, dt);
}

template<>
MOTION_MODEL_PUBLIC CatrMotionModel32::State::Matrix CatrMotionModel32::crtp_jacobian(
  const CatrMotionModel32::State & state,
  const std::chrono::nanoseconds & dt) const
{
  return catr_jacobian<float32_t>(state, dt);
}

template<>
MOTION_MODEL_PUBLIC CatrMotionModel64::State::Matrix CatrMotionModel64::crtp_jacobian(
  const CatrMotionModel64::State & state,
  const std::chrono::nanoseconds & dt) const
{
  return catr_jacobian<float64_t>(state, dt);
}

}  // namespace motion_model
}  // namespace common
}  // namespace autoware
