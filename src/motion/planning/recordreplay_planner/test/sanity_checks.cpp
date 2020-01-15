// Copyright 2019 Christopher Ho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <recordreplay_planner/recordreplay_planner.hpp>

#include <chrono>

using motion::planning::recordreplay_planner::RecordReplayPlanner;
using std::chrono::system_clock;

class sanity_checks_base : public ::testing::Test
{
protected:
  RecordReplayPlanner planner_{};
};

struct MyParameters
{
  double some_parameter;
};

class sanity_checks_constraint_free
  : public sanity_checks_base, public testing::WithParamInterface<MyParameters>
{
};

TEST_P(sanity_checks_constraint_free, sometest)
{
  const auto t = system_clock::now();
  auto p = GetParam();
  EXPECT_EQ(1, 1);
}

INSTANTIATE_TEST_CASE_P(
  constraint_free,
  sanity_checks_constraint_free,
  testing::Values(
    MyParameters{0.0},
    MyParameters{0.1}
));
