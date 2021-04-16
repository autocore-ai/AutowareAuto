// Copyright 2021 the Autoware Foundation
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

/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief Implements the linear motion model.

#include <motion_model/differential_drive_motion_model.hpp>

#include <common/types.hpp>
#include <helper_functions/float_comparisons.hpp>

#include <cmath>

namespace
{
constexpr auto kEpsilon = 0.00001F;

using std::cos;
using std::sin;

using autoware::prediction::variable::X;
using autoware::prediction::variable::Y;
using autoware::prediction::variable::YAW;
using autoware::prediction::variable::YAW_CHANGE_RATE;
using autoware::prediction::variable::YAW_CHANGE_ACCELERATION;
using autoware::prediction::variable::XY_VELOCITY;
using autoware::prediction::variable::XY_ACCELERATION;
using autoware::common::types::float32_t;

}  // namespace

namespace autoware
{
namespace prediction
{

template<>
CvtrMotionModel::State CvtrMotionModel::crtp_predict(
  const CvtrMotionModel::State & state,
  const std::chrono::nanoseconds & dt) const
{
  CvtrMotionModel::State new_state{};
  const auto t = std::chrono::duration<float32_t>{dt}.count();
  const auto theta = state.at<YAW>();
  const auto x = state.at<X>();
  const auto y = state.at<Y>();
  const auto v = state.at<XY_VELOCITY>();
  const auto w = state.at<YAW_CHANGE_RATE>();
  if (common::helper_functions::comparisons::abs_eq_zero(w, kEpsilon)) {
    new_state.at<X>() = x + t * v * cos(theta);
    new_state.at<Y>() = y + t * v * sin(theta);
  } else {
    new_state.at<X>() = x + v * (-sin(theta) + sin(t * w + theta)) / w;
    new_state.at<Y>() = y + v * (cos(theta) - cos(t * w + theta)) / w;
  }
  new_state.at<XY_VELOCITY>() = v;
  new_state.at<YAW>() = t * w + theta;
  new_state.at<YAW_CHANGE_RATE>() = w;
  return new_state;
}

template<>
CvtrMotionModel::State::Matrix CvtrMotionModel::crtp_jacobian(
  const State & state,
  const std::chrono::nanoseconds & dt) const
{
  CvtrMotionModel::State::Matrix jacobian{CvtrMotionModel::State::Matrix::Identity()};
  const auto t = std::chrono::duration<float32_t>{dt}.count();
  const auto theta = state.at<YAW>();
  const auto v = state.at<XY_VELOCITY>();
  const auto w = state.at<YAW_CHANGE_RATE>();
  const auto x_index = State::index_of<X>();
  const auto y_index = State::index_of<Y>();
  const auto v_index = State::index_of<XY_VELOCITY>();
  const auto theta_index = State::index_of<YAW>();
  const auto w_index = State::index_of<YAW_CHANGE_RATE>();
  if (common::helper_functions::comparisons::abs_eq_zero(w, kEpsilon)) {
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
    const auto inv_w = 1.0F / w;
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


template<>
CatrMotionModel::State CatrMotionModel::crtp_predict(
  const CatrMotionModel::State & state,
  const std::chrono::nanoseconds & dt) const
{
  CatrMotionModel::State new_state{};
  const auto t = std::chrono::duration<float32_t>{dt}.count();
  const auto theta = state.at<YAW>();
  const auto x = state.at<X>();
  const auto y = state.at<Y>();
  const auto v = state.at<XY_VELOCITY>();
  const auto a = state.at<XY_ACCELERATION>();
  const auto w = state.at<YAW_CHANGE_RATE>();
  const auto half_t_squared_a = 0.5F * t * t * a;
  if (common::helper_functions::comparisons::abs_eq_zero(w, kEpsilon)) {
    new_state.at<X>() = x + half_t_squared_a * cos(theta) + t * v * cos(theta);
    new_state.at<Y>() = y + half_t_squared_a * sin(theta) + t * v * sin(theta);
  } else {
    const auto t_w_plus_theta = t * w + theta;
    const auto inv_w = 1.0F / w;
    const auto inv_w_2 = inv_w * inv_w;
    new_state.at<X>() = x +
      inv_w * t * a * sin(t_w_plus_theta) -
      inv_w * v * sin(theta) +
      inv_w * v * sin(t_w_plus_theta) +
      inv_w_2 * a * (-cos(theta) + cos(t_w_plus_theta));
    new_state.at<Y>() = y -
      inv_w * t * a * cos(t_w_plus_theta) +
      inv_w * v * cos(theta) -
      inv_w * v * cos(t_w_plus_theta) +
      inv_w_2 * a * (-sin(theta) + sin(t_w_plus_theta));
  }
  new_state.at<YAW>() = theta + w * t;
  new_state.at<YAW_CHANGE_RATE>() = w;
  new_state.at<XY_VELOCITY>() = v + t * a;
  new_state.at<XY_ACCELERATION>() = a;
  return new_state;
}

template<>
CatrMotionModel::State::Matrix CatrMotionModel::crtp_jacobian(
  const State & state,
  const std::chrono::nanoseconds & dt) const
{
  CatrMotionModel::State::Matrix jacobian{CatrMotionModel::State::Matrix::Identity()};
  const auto t = std::chrono::duration<float32_t>{dt}.count();
  const auto theta = state.at<YAW>();
  const auto v = state.at<XY_VELOCITY>();
  const auto a = state.at<XY_ACCELERATION>();
  const auto w = state.at<YAW_CHANGE_RATE>();
  const auto x_index = State::index_of<X>();
  const auto y_index = State::index_of<Y>();
  const auto theta_index = State::index_of<YAW>();
  const auto v_index = State::index_of<XY_VELOCITY>();
  const auto w_index = State::index_of<YAW_CHANGE_RATE>();
  const auto a_index = State::index_of<XY_ACCELERATION>();
  if (common::helper_functions::comparisons::abs_eq_zero(w, kEpsilon)) {
    // X row non-identity entries
    jacobian(x_index, theta_index) = -0.5F * t * t * a * sin(theta) - t * v * sin(theta);
    jacobian(x_index, v_index) = t * cos(theta);
    jacobian(x_index, w_index) = -0.5F * t * t * t * a * sin(theta) - t * t * v * sin(theta);
    jacobian(x_index, a_index) = 0.5F * t * t * cos(theta);
    // Y row non-identity entries
    jacobian(y_index, theta_index) = 0.5F * t * t * a * cos(theta) + t * v * cos(theta);
    jacobian(y_index, v_index) = t * sin(theta);
    jacobian(y_index, w_index) = 0.5F * t * t * t * a * cos(theta) + t * t * v * cos(theta);
    jacobian(y_index, a_index) = 0.5F * t * t * sin(theta);
    // Velocity row non-identity entries
    jacobian(v_index, a_index) = t;
  } else {
    const auto inv_w = 1.0F / w;
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
    jacobian(x_index, theta_index) =
      t * a * inv_w * cos_next_theta + v_cos_theta_diff_inv_w - a * inv_w * sin_theta_diff_inv_w;
    jacobian(x_index, v_index) = sin_theta_diff_inv_w;
    jacobian(x_index, w_index) =
      t * t * a * inv_w * cos_next_theta +
      t * v * inv_w * cos_next_theta -
      2.0F * t * a * inv_w * inv_w * sin_next_theta -
      inv_w * v_sin_theta_diff_inv_w -
      2.0F * a * inv_w * inv_w * cos_theta_diff_inv_w;
    jacobian(x_index, a_index) = t * inv_w * sin_next_theta + inv_w * cos_theta_diff_inv_w;
    // Y row non-identity entries.
    jacobian(y_index, theta_index) =
      t * a * inv_w * sin_next_theta + v_sin_theta_diff_inv_w + a * inv_w * cos_theta_diff_inv_w;
    jacobian(y_index, v_index) = -cos_theta_diff_inv_w;
    jacobian(y_index, w_index) =
      t * t * a * inv_w * sin_next_theta +
      t * v * inv_w * sin_next_theta +
      2.0F * t * a * inv_w * inv_w * cos_next_theta +
      inv_w * v_cos_theta_diff_inv_w -
      2.0F * a * inv_w * inv_w * sin_theta_diff_inv_w;
    jacobian(y_index, a_index) = -t * inv_w * cos_next_theta + inv_w * sin_theta_diff_inv_w;
    // Velocity row non-identity entries.
    jacobian(v_index, a_index) = t;
    // Theta changes linearly with turn rate.
    jacobian(theta_index, w_index) = t;
  }
  return jacobian;
}

}  // namespace prediction
}  // namespace autoware
