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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>

#include <measurement_conversion/measurement_transformation.hpp>

using autoware::common::state_estimation::downscale_isometry;

/// \test Check that we can downscale a trivial isometry.
TEST(Measurement2dConversionTest, trivial_isometry_downscaling) {
  const auto isometry = downscale_isometry<2>(Eigen::Isometry3f::Identity());
  EXPECT_TRUE(isometry.isApprox(Eigen::Isometry2f::Identity()));
}

/// \test Check that we can downscale an isometry.
TEST(Measurement2dConversionTest, isometry_downscaling) {
  Eigen::Isometry3f initial_isometry;
  initial_isometry.linear() =
    Eigen::AngleAxisf{autoware::common::types::PI, Eigen::Vector3f::UnitZ()} *Eigen::Scaling(1.0F);
  const auto isometry = downscale_isometry<2>(initial_isometry);
  Eigen::Matrix2f expected_rotation = Eigen::Rotation2Df(autoware::common::types::PI) *
    Eigen::Scaling(1.0F);
  EXPECT_TRUE(isometry.linear().isApprox(expected_rotation));
}
