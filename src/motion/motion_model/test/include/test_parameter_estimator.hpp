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

// Copyright 2018 Apex.AI, Inc.
// All rights reserved.

#include <motion_model/parameter_estimator.hpp>

using Eigen::Matrix;
using autoware::motion::motion_model::ParameterEstimator;

TEST(motion_model, parameter_estimator)
{
  ParameterEstimator<2> model;
  Matrix<float, 2, 1> x({1, 2}), y, z;
  Matrix<float, 2, 2> F;

  model.reset(x);

  std::chrono::nanoseconds dt(100LL);
  model.compute_jacobian_and_predict(F, dt);
  y = model.get_state();
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      if (i == j) {
        ASSERT_FLOAT_EQ(F(i, j), 1.0F);
      } else {
        ASSERT_FLOAT_EQ(F(i, j), 0.0F);
      }
    }
  }
  ASSERT_FLOAT_EQ(y(0), 1.0F);
  ASSERT_FLOAT_EQ(y(1), 2.0F);
  // predict
  ASSERT_FLOAT_EQ(model[0], 1.0F);
  ASSERT_FLOAT_EQ(model[1], 2.0F);
  model.predict(z, dt);
  ASSERT_FLOAT_EQ(z(0), 1.0F);
  ASSERT_FLOAT_EQ(z(1), 2.0F);
  model.predict(dt);
  ASSERT_FLOAT_EQ(model[0], 1.0F);
  ASSERT_FLOAT_EQ(model[1], 2.0F);
}
