// Copyright 2018 Apex.AI, Inc.
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

/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.

#include <common/types.hpp>
#include "motion_model/catr_core.hpp"

using autoware::common::types::float32_t;

namespace autoware
{
namespace motion
{
namespace motion_model
{
/////
template<int32_t NumStates>
void catr_workspace_init_invariant(
  const Eigen::Matrix<float32_t, NumStates, 1U> & x,
  CatrInvariantWorkspace & ws)
{
  static_assert(NumStates >= 6U, "CATR model and derivatives must be at least 6-dimensional");
  // time invariant
  ws.a = x(CatrState::ACCELERATION);
  ws.w = x(CatrState::TURN_RATE);
  const float32_t th = x(CatrState::HEADING);
  ws.s = sinf(th);
  ws.c = cosf(th);
  // TODO(c.ho) sin/cos lookup table?
  ws.is_w_nonzero = (fabsf(ws.w) > 0.001F);  // TODO(c.ho) param
  if (ws.is_w_nonzero) {
    ws.vw = x(CatrState::VELOCITY) * ws.w;
    ws.w_inv = 1.0F / ws.w;
    ws.w2_inv = ws.w_inv * ws.w_inv;
  }
}

/////
template<int32_t NumStates>
void catr_workspace_init_variant(
  const Eigen::Matrix<float32_t, NumStates, 1U> & x,
  const float32_t dt_s,
  const CatrInvariantWorkspace & iws,
  CatrVariantWorkspace & ws)
{
  static_assert(NumStates >= 6U, "CATR model and derivatives must be at least 6-dimensional");
  // varies wrt dt
  ws.vp = x(CatrState::VELOCITY) + (dt_s * iws.a);
  ws.dt = dt_s;
  // slight abuse of notation to avoid an else branch TODO(c.ho) benchmark
  // This term is equivalent to 0.5 * a * t * t + v * t = ds
  // == ((v + a * t) + v) * 0.5 * dt == 0.5 * dt * (2 v + a * t)
  ws.awT = (ws.vp + x(CatrState::VELOCITY)) * (dt_s * 0.5F);
  ws.thp = x(CatrState::HEADING);
  if (iws.is_w_nonzero) {
    ws.wT = iws.w * dt_s;
    ws.thp += ws.wT;
    ws.sp = sinf(ws.thp);
    ws.cp = cosf(ws.thp);
    ws.awT = ws.wT * iws.a;
  }
}

/////
template<int32_t NumStates>
void catr_compute_jacobian(
  const CatrInvariantWorkspace & iws,
  const CatrVariantWorkspace & vws,
  Eigen::Matrix<float32_t, NumStates, NumStates> & F)
{
  static_assert(NumStates >= 6U, "CATR model and derivatives must be at least 6-dimensional");
  // identity matrix
  F.setIdentity();
  //// easy linear terms
  F(CatrState::VELOCITY, CatrState::ACCELERATION) =
    vws.dt;
  F(CatrState::HEADING, CatrState::TURN_RATE) =
    vws.dt;
  //// hard position terms: computed with SymPy
  if (iws.is_w_nonzero) {
    // * / dv
    const float32_t ds = vws.sp - iws.s;
    F(CatrState::POSE_X, CatrState::VELOCITY) =
      iws.w_inv * (ds);
    const float32_t dc = vws.cp - iws.c;
    F(CatrState::POSE_Y, CatrState::VELOCITY) =
      iws.w_inv * (-dc);
    // * / da
    const float32_t wT_sp = vws.wT * vws.sp;
    F(CatrState::POSE_X, CatrState::ACCELERATION) =
      iws.w2_inv * (wT_sp + dc);
    const float32_t wT_cp = vws.wT * vws.cp;
    F(CatrState::POSE_Y, CatrState::ACCELERATION) =
      iws.w2_inv * (ds - wT_cp);
    // * / dth
    F(CatrState::POSE_X, CatrState::HEADING) =
      iws.w2_inv * ((iws.a * (-ds)) + (((iws.w * vws.vp) * vws.cp) - (iws.vw * iws.c)));
    F(CatrState::POSE_Y, CatrState::HEADING) =
      iws.w2_inv * ((iws.a * (dc)) + (((iws.w * vws.vp) * vws.sp) - (iws.vw * iws.s)));
    // * / dw
    const float32_t w3_inv = iws.w2_inv * iws.w_inv;
    F(CatrState::POSE_X, CatrState::TURN_RATE) =
      w3_inv * (
      (vws.wT * ((iws.w * vws.vp * vws.cp) - (2.0F * iws.a * vws.sp))) -
      ((2.0F * iws.a * dc) + (iws.vw * ds))
      );
    F(CatrState::POSE_Y, CatrState::TURN_RATE) =
      w3_inv * (
      ((iws.vw * dc) - (2.0F * iws.a * ds)) +
      (vws.wT * ((iws.w * vws.vp * vws.sp) + (2.0F * iws.a * vws.cp)))
      );
  } else {
    // * / dv
    F(CatrState::POSE_X, CatrState::VELOCITY) =
      vws.dt * iws.c;
    F(CatrState::POSE_Y, CatrState::VELOCITY) =
      vws.dt * iws.s;
    // * / da
    const float32_t dt2_2 = 0.5F * vws.dt * vws.dt;
    F(CatrState::POSE_X, CatrState::ACCELERATION) =
      dt2_2 * iws.c;
    F(CatrState::POSE_Y, CatrState::ACCELERATION) =
      dt2_2 * iws.s;
    // * / dth
    // vt + 0.5 at^2 == ds == ws.awT
    F(CatrState::POSE_X, CatrState::HEADING) =
      -iws.s * vws.awT;
    F(CatrState::POSE_Y, CatrState::HEADING) =
      iws.c * vws.awT;
  }
}

/////
template<int32_t NumStates>
void catr_predict(
  const Eigen::Matrix<float32_t, NumStates, 1U> & ref,
  const CatrInvariantWorkspace & iws,
  const CatrVariantWorkspace & vws,
  Eigen::Matrix<float32_t, NumStates, 1U> & x)
{
  static_assert(NumStates >= 6U, "CATR model and derivatives must be at least 6-dimensional");
  if (iws.is_w_nonzero) {
    const float32_t vw_awt = iws.vw + vws.awT;
    x(CatrState::POSE_X) = ref(CatrState::POSE_X) + (iws.w2_inv *
      ((vw_awt * vws.sp) +
      ((iws.a * (vws.cp - iws.c)) -
      ((iws.vw * iws.s)))));
    x(CatrState::POSE_Y) = ref(CatrState::POSE_Y) + (iws.w2_inv *
      ((-vw_awt * vws.cp) +
      (iws.a * (vws.sp - iws.s)) +
      (iws.vw * iws.c)));
  } else {
    // vt + 0.5 at^2 == ds == ws.awT
    x(CatrState::POSE_X) = ref(CatrState::POSE_X) + (vws.awT * iws.c);
    x(CatrState::POSE_Y) = ref(CatrState::POSE_Y) + (vws.awT * iws.s);
  }
  x(CatrState::VELOCITY) = vws.vp;
  x(CatrState::ACCELERATION) = iws.a;
  x(CatrState::HEADING) = vws.thp;
  x(CatrState::TURN_RATE) = iws.w;
}

////////////////////////////////////////////////////////////////////////////////////////////////
/////// Template instantiation ////////
// N = 6
template void catr_workspace_init_invariant<6U>(
  const Eigen::Matrix<float32_t, 6U, 1U> &,
  CatrInvariantWorkspace &);
template void catr_workspace_init_variant<6U>(
  const Eigen::Matrix<float32_t, 6U, 1U> &,
  const float32_t,
  const CatrInvariantWorkspace &,
  CatrVariantWorkspace &);
template void catr_compute_jacobian<6U>(
  const CatrInvariantWorkspace &,
  const CatrVariantWorkspace &,
  Eigen::Matrix<float32_t, 6U, 6U> &);
template void catr_predict<6U>(
  const Eigen::Matrix<float32_t, 6U, 1U> &,
  const CatrInvariantWorkspace &,
  const CatrVariantWorkspace &,
  Eigen::Matrix<float32_t, 6U, 1U> &);
// N = 8
template void catr_workspace_init_invariant<8U>(
  const Eigen::Matrix<float32_t, 8U, 1U> &,
  CatrInvariantWorkspace &);
template void catr_workspace_init_variant<8U>(
  const Eigen::Matrix<float32_t, 8U, 1U> &,
  const float32_t,
  const CatrInvariantWorkspace &,
  CatrVariantWorkspace &);
template void catr_compute_jacobian<8U>(
  const CatrInvariantWorkspace &,
  const CatrVariantWorkspace &,
  Eigen::Matrix<float32_t, 8U, 8U> &);
template void catr_predict<8U>(
  const Eigen::Matrix<float32_t, 8U, 1U> &,
  const CatrInvariantWorkspace &,
  const CatrVariantWorkspace &,
  Eigen::Matrix<float32_t, 8U, 1U> &);
}  // namespace motion_model
}  // namespace motion
}  // namespace autoware
