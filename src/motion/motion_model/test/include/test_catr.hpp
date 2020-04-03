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

#include "motion_model/catr_model.hpp"

using autoware::motion::motion_model::CatrModel;
using autoware::motion::motion_model::CatrState;
using Eigen::Matrix;

static const float TOL = 1.0E-5F;

// catr model should safely decay to constant velocity
TEST(catr_model, constant_velocity)
{
  CatrModel model;
  // position at origin, unit velocity, no acceleration/yaw rate
  Matrix<float, 6, 1> x, y;
  x << 0, 0, 1, 0, 0, 0;
  model.reset(x);
  // duration in nanoseconds equal to one second
  std::chrono::seconds dt(1);
  model.predict(y, dt);
  ASSERT_FLOAT_EQ(y(CatrState::POSE_X), 1.0);
  ASSERT_FLOAT_EQ(y(CatrState::POSE_Y), 0);
  ASSERT_FLOAT_EQ(y(CatrState::VELOCITY), 1);
  ASSERT_FLOAT_EQ(y(CatrState::ACCELERATION), 0);
  ASSERT_FLOAT_EQ(y(CatrState::HEADING), 0);
  ASSERT_FLOAT_EQ(y(CatrState::TURN_RATE), 0);
}


// catr model should handle constant acceleration
TEST(catr_model, constant_acceleration)
{
  CatrModel model;
  // position at origin, unit velocity, unit acceleration, no yaw rate
  const float th = 3.14159 / 2.0F;
  Matrix<float, 6, 1> x;
  x << 0, 0, 1, 1, th, 0;
  model.reset(x);

  // duration in nanoseconds equal to one second
  std::chrono::seconds dt(1);
  model.predict(dt);
  ASSERT_LT(fabsf(model[CatrState::POSE_X] - 0), TOL);
  ASSERT_LT(fabsf(model[CatrState::POSE_Y] - 1.5), TOL);
  ASSERT_LT(fabsf(model[CatrState::VELOCITY] - 2), TOL);
  ASSERT_LT(fabsf(model[CatrState::ACCELERATION] - 1), TOL);
  ASSERT_LT(fabsf(model[CatrState::HEADING] - th), TOL);
  ASSERT_LT(fabsf(model[CatrState::TURN_RATE] - 0), TOL);

}


// handle constant velocity, constant turn rate = circle
TEST(catr_model, cvtr1)
{
  CatrModel model;
  // i should be able to simulate a full circle
  const uint32_t N = 100;
  const float dt = 0.1;
  // full rotation in 100 steps = 10 seconds
  const float dth = 2.0F * 3.14159F / (dt * N);
  const float v = 2.0F;
  // travel circumference in 10 seconds
  const float s = v * (dt * N);
  const float r = s / (2.0F * 3.14159F);
  // 0 heading == +x direction
  Matrix<float, 6, 1> x;
  x << 0, -r, v, 0, 0, dth;

  model.reset(x);
  std::chrono::nanoseconds dt_seconds(static_cast<long>(dt * 1000.0F * 1000000.0F));
  for (uint32_t i = 0; i < N; ++i) {
    model.predict(dt_seconds);
    const float th = ((i + 1) * dth) / (1 / dt);
    const float x = r * sinf(th);
    const float y = -r * cosf(th);
    const float TOL2 = 1.0E-4F;
    ASSERT_LT(fabsf(model[CatrState::POSE_X] - x), TOL2);
    ASSERT_LT(fabsf(model[CatrState::POSE_Y] - y), TOL2);
    ASSERT_LT(fabsf(model[CatrState::VELOCITY] - v), TOL2);
    ASSERT_LT(fabsf(model[CatrState::ACCELERATION]), TOL2);
    ASSERT_LT(fabsf(model[CatrState::HEADING] - th), TOL2);
    ASSERT_LT(fabsf(model[CatrState::TURN_RATE] - dth), TOL2);
  }
}
// same thing as above, spinning other way
TEST(catr_model, cvtr2)
{
  CatrModel model;
  // i should be able to simulate a full circle
  const uint32_t N = 200;
  const float dt = 0.1;
  // full rotation in 100 steps = 10 seconds
  const float dth = -2.0F * 3.14159F / (dt * N);
  const float v = 3.0F;
  // travel circumference in 10 seconds
  const float s = v * (dt * N);
  const float r = s / (2.0F * 3.14159F);
  const float th0 = 3.14159F;
  // 0 heading == +x direction
  Matrix<float, 6, 1> x;
  x << 0, -r, v, 0, th0, dth;

  model.reset(x);

  std::chrono::nanoseconds dt_seconds(static_cast<long>(dt * 1000.0F * 1000000.0F));
  for (uint32_t i = 0; i < N; ++i) {
    model.predict(dt_seconds);
    const float th = ((i + 1) * dth) / (1.0 / dt) + th0;
    const float x = -r * sinf(th);
    const float y = r * cosf(th);
    const float TOL2 = 1.0E-4F;
    ASSERT_LT(fabsf(model[CatrState::POSE_X] - x), TOL2);
    ASSERT_LT(fabsf(model[CatrState::POSE_Y] - y), TOL2);
    ASSERT_LT(fabsf(model[CatrState::VELOCITY] - v), TOL2);
    ASSERT_LT(fabsf(model[CatrState::ACCELERATION]), TOL2);
    ASSERT_LT(fabsf(model[CatrState::HEADING] - th), TOL2);
    ASSERT_LT(fabsf(model[CatrState::TURN_RATE] - dth), TOL2);
  }
}
// basic sanity checks
TEST(catr_model, basic)
{
  // positive yaw rate should put position away from initial heading, and continue on
  CatrModel model;
  // general case
  float th = 0.0F;
  Matrix<float, 6, 1> z1, z2;
  z1 << 0, 0, 1, 1, th, 0.1F;
  model.reset(z1);

  std::chrono::nanoseconds dt_seconds_100(std::chrono::milliseconds(100));
  float x = model[CatrState::POSE_X];
  float y = model[CatrState::POSE_Y];
  model.predict(z2, dt_seconds_100);
  ASSERT_GT(z2(CatrState::POSE_X), 0.0F);
  ASSERT_GT(z2(CatrState::POSE_Y), 0.0F);
  ASSERT_LT(fabsf(z2(CatrState::HEADING) - 0.1 * 0.1), TOL);
  ASSERT_LT(fabsf(z2(CatrState::VELOCITY) - 1.1F), TOL);
  ASSERT_LT(fabsf(z2(CatrState::ACCELERATION) - 1.0F), TOL);
  float xp = z2(CatrState::POSE_X);
  float yp = z2(CatrState::POSE_Y);
  float thp = atan2f(yp - y, xp - x);
  ASSERT_GT(thp, th);
  x = xp;
  y = yp;
  th = thp;

  std::chrono::nanoseconds dt_seconds_200(std::chrono::milliseconds(200));
  model.predict(z2, dt_seconds_200);
  ASSERT_GT(z2(CatrState::POSE_X), x);
  ASSERT_GT(z2(CatrState::POSE_Y), y);
  ASSERT_LT(fabsf(z2(CatrState::HEADING) - 0.2 * 0.1), TOL);
  ASSERT_LT(fabsf(z2(CatrState::VELOCITY) - 1.2F), TOL);
  ASSERT_LT(fabsf(z2(CatrState::ACCELERATION) - 1.0F), TOL);
  xp = z2(CatrState::POSE_X);
  yp = z2(CatrState::POSE_Y);
  thp = atan2f(yp - y, xp - x);
  ASSERT_GT(thp, th);
}

// check correctness of jacobian in constant velocity case
TEST(catr_model, cv_jacobian)
{
  CatrModel model;
  // position at origin, unit velocity, no acceleration/yaw rate
  Matrix<float, 6, 6> F;
  // 30 degrees
  const float v = 1.0;
  Matrix<float, 6, 1> x, y;
  x << 0, 0, v, 0, 30 * 3.14159F / 180, 0.0F;
  model.reset(x);

  std::chrono::nanoseconds dt_(std::chrono::milliseconds(100));
  const float dt = static_cast<float>(dt_.count()) / 1000000000LL;
  const float ds = dt * v;
  model.compute_jacobian_and_predict(F, dt_);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i == j) {
        // diagonal
        ASSERT_LT(fabsf(F(i, i) - 1.0F), TOL);
      } else if (i == 0 && j == 2) {
        // dx/dv
        ASSERT_LT(fabsf(F(i, j) - dt * sqrtf(3) * 0.5), TOL);
      } else if (i == 0 && j == 3) {
        // dx/da
        ASSERT_LT(fabsf(F(i, j) - dt * dt * sqrtf(3) * 0.25), TOL);
      } else if (i == 0 && j == 4) {
        // dx/dth
        ASSERT_LT(fabsf(F(i, j) - -0.5 * ds), TOL);
      } else if (i == 1 && j == 2) {
        // dy/dv
        ASSERT_LT(fabsf(F(i, j) - dt * 0.5), TOL);
      } else if (i == 1 && j == 3) {
        // dy/da
        ASSERT_LT(fabsf(F(i, j) - dt * dt * 0.25), TOL);
      } else if (i == 1 && j == 4) {
        // dy/dth
        ASSERT_LT(fabsf(F(i, j) - (ds * sqrtf(3) * 0.5)), TOL);
      } else if (i == 2 && j == 3) {
        // dv/da
        ASSERT_LT(fabsf(F(i, j) - (dt)), TOL);
      } else if (i == 4 && j == 5) {
        // dth/dw
        ASSERT_LT(fabsf(F(i, j) - (dt)), TOL);
      } else {
        ASSERT_LT(fabsf(F(i, j) - 0.0F), TOL) << i << ", " << j;
      }
    }
  }
  ASSERT_LT(fabsf(model[CatrState::POSE_X] - dt * sqrtf(3) * 0.5), TOL);
  ASSERT_LT(fabsf(model[CatrState::POSE_Y] - 0.05), TOL);
  ASSERT_LT(fabsf(model[CatrState::VELOCITY] - 1), TOL);
  ASSERT_LT(fabsf(model[CatrState::ACCELERATION] - 0), TOL);
  ASSERT_LT(fabsf(model[CatrState::HEADING] - 3.14159 / 6), TOL);
  ASSERT_LT(fabsf(model[CatrState::TURN_RATE] - 0), TOL);
}

// check correctness of jacobian in constant acceleration case
TEST(catr_model, ca_jacobian)
{
  CatrModel model;
  // position at origin, unit velocity, unit acceleration, no yaw rate
  Matrix<float, 6, 6> F;
  // -45 degrees
  const float v = 1.0;
  Matrix<float, 6, 1> x, y;
  x << 2, -3, v, 1, -3.14159F / 4, 0.0F;
  model.reset(x);

  std::chrono::nanoseconds dt_(std::chrono::milliseconds(200));
  const float dt = static_cast<float>(dt_.count()) / 1000000000LL;
  const float ds = dt * v + 0.5F * dt * dt;
  const float cs = sqrtf(2) * 0.5F;
  model.compute_jacobian(F, dt_);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i == j) {
        // diagonal
        ASSERT_LT(fabsf(F(i, i) - 1.0F), TOL);
      } else if (i == 0 && j == 2) {
        // dx/dv
        ASSERT_LT(fabsf(F(i, j) - dt * cs), TOL);
      } else if (i == 0 && j == 3) {
        // dx/da
        ASSERT_LT(fabsf(F(i, j) - dt * dt * cs * 0.5), TOL);
      } else if (i == 0 && j == 4) {
        // dx/dth
        ASSERT_LT(fabsf(F(i, j) - cs * ds), TOL);
      } else if (i == 1 && j == 2) {
        // dy/dv
        ASSERT_LT(fabsf(F(i, j) + dt * cs), TOL);
      } else if (i == 1 && j == 3) {
        // dy/da
        ASSERT_LT(fabsf(F(i, j) + dt * dt * 0.5 * cs), TOL);
      } else if (i == 1 && j == 4) {
        // dy/dth
        ASSERT_LT(fabsf(F(i, j) - (ds * cs)), TOL);
      } else if (i == 2 && j == 3) {
        // dv/da
        ASSERT_LT(fabsf(F(i, j) - (dt)), TOL);
      } else if (i == 4 && j == 5) {
        // dth/dw
        ASSERT_LT(fabsf(F(i, j) - (dt)), TOL);
      } else {
        ASSERT_LT(fabsf(F(i, j) - 0.0F), TOL) << i << ", " << j;
      }
    }
  }
  model.predict(dt_);
  ASSERT_LT(fabsf(model[CatrState::POSE_X] - (2 + cs * ds)), TOL);
  ASSERT_LT(fabsf(model[CatrState::POSE_Y] - (-3 - cs * ds)), TOL);
  ASSERT_LT(fabsf(model[CatrState::VELOCITY] - 1.2), TOL);
  ASSERT_LT(fabsf(model[CatrState::ACCELERATION] - 1), TOL);
  ASSERT_LT(fabsf(model[CatrState::HEADING] + 3.14159 / 4), TOL);
  ASSERT_LT(fabsf(model[CatrState::TURN_RATE] - 0), TOL);

}

// check correctness of jacobian in constant velocity, constant turn rate case
// jacobian estimated via finite difference
TEST(catr_model, cvtr_jacobian)
{
  CatrModel model;
  // constant velocity, turn rate model, path = circle
  Matrix<float, 6, 6> F;
  // -45 degrees
  const Matrix<float, 6, 1> x((Eigen::Matrix<float, 6, 1>() << -1, -3, 5.0F, 0, -3.14159F * 1.5F, 0.1F).finished());
  Matrix<float, 6, 1> dx[6];

  std::chrono::nanoseconds dt_(1000000LL);
  // compute deltas
  const float del = 0.01;
  for (int i = 0; i < 6; ++i) {
    dx[i] = x;
    dx[i](i) += del;
    model.reset(dx[i]);
    model.predict(dt_);
    dx[i] = model.get_state();
  }
  model.reset(x);
  model.compute_jacobian_and_predict(F, dt_);

  // compare analytical derivative to numerical derivative
  const float TOL2 = 3.0E-4F;
  ASSERT_LT(TOL2, del);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i == j) {
        // diagonal is 1
        ASSERT_EQ(F(i, j), 1.0F);
      } else if (i > j) {
        // subdiagonal is 0
        ASSERT_LT(fabsf(F(i, j)), TOL);
      } else {
        // should be approximately the same
        const float delta = (dx[j](i) - model[i]) / del;
        ASSERT_LT(fabsf(delta - F(i, j)), TOL2) << i << ", " << j << ": del = " << delta;
        // check for matching sign
        if (fabsf(F(i, j)) > TOL2) {
          ASSERT_FLOAT_EQ(std::copysign(1.0F, F(i, j)), std::copysign(1.0F, delta)) << i << ", " << j;
        }
      }
      // const float delta2 = (dx[j](i) - model[i]) / del;
      // std::cerr << "(" << F(i, j) << ", " << delta2 << "),\t";
    }
    // std::cerr << "\n";
  }
}

// check correctness of jacobian in general case
TEST(catr_model, catr_jacobian)
{
  CatrModel model;
  // general case
  Matrix<float, 6, 6> F;
  // -45 degrees
  const Matrix<float, 6, 1> x((Eigen::Matrix<float, 6, 1>() <<-10, 3, 5, 3, -0.5F, -0.3F).finished());
  Matrix<float, 6, 1> dx[6];
  std::chrono::nanoseconds dt_(1000000LL);
  // compute deltas
  const float del = 0.01;
  for (int i = 0; i < 6; ++i) {
    dx[i] = x;
    dx[i](i) += del;
    model.reset(dx[i]);
    model.predict(dt_);
    dx[i] = model.get_state();
  }
  model.reset(x);
  model.compute_jacobian_and_predict(F, dt_);

  // compare analytical derivative to numerical derivative
  const float TOL2 = 3.0E-4F;
  ASSERT_LT(TOL2, del);
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i == j) {
        // diagonal is 1
        ASSERT_EQ(F(i, j), 1.0F);
      } else if (i > j) {
        // subdiagonal is 0
        ASSERT_LT(fabsf(F(i, j)), TOL);
      } else {
        // should be approximately the same
        const float delta = (dx[j](i) - model[i]) / del;
        ASSERT_LT(fabsf(delta - F(i, j)), TOL2) << i << ", " << j << ": del = " << delta;
        // check for matching sign
        if (fabsf(F(i, j)) > TOL2) {
          ASSERT_FLOAT_EQ(std::copysign(1.0F, F(i, j)), std::copysign(1.0F, delta)) << i << ", " << j;
        }
      }
      // const float delta2 = (dx[j](i) - model[i]) / del;
      // std::cerr << "(" << F(i, j) << ", " << delta2 << "),\t";
    }
    // std::cerr << "\n";
  }

}
