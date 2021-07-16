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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_
#define MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_

#include <Eigen/Geometry>


namespace autoware
{
namespace common
{
namespace state_estimation
{

///
/// @brief      Downscale the isometry to a lower dimension if needed.
///
/// @param[in]  isometry              The isometry transform
///
/// @tparam     kStateDimensionality  Dimensionality of the space.
/// @tparam     FloatT                Type of scalar.
///
/// @return     Downscaled isometry.
///
template<std::int32_t kStateDimensionality, typename FloatT>
static constexpr Eigen::Transform<
  FloatT, kStateDimensionality, Eigen::TransformTraits::Isometry> downscale_isometry(
  const Eigen::Transform<FloatT, 3, Eigen::TransformTraits::Isometry> & isometry)
{
  static_assert(kStateDimensionality <= 3, "We only handle scaling the isometry down.");
  using Isometry = Eigen::Transform<
    FloatT, kStateDimensionality, Eigen::TransformTraits::Isometry>;
  Isometry result{Isometry::Identity()};
  result.linear() = isometry.rotation()
    .template block<kStateDimensionality, kStateDimensionality>(0, 0);
  result.translation() = isometry.translation().topRows(kStateDimensionality);
  return result;
}

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware


#endif  // MEASUREMENT_CONVERSION__EIGEN_UTILS_HPP_
