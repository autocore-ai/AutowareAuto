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

/// \copyright Copyright 2021 Apex.AI, Inc.
/// All rights reserved.

#include <gtest/gtest.h>

#include <state_estimation_nodes/steady_time_grid.hpp>

using autoware::prediction::SteadyTimeGrid;

/// \test Check the initialization status.
TEST(SteadyTimeGridTest, Initialization) {
  SteadyTimeGrid time_grid;
  EXPECT_FALSE(time_grid.is_initialized());
  const std::chrono::system_clock::time_point start{std::chrono::seconds{42LL}};
  const std::chrono::system_clock::duration interval{std::chrono::milliseconds{50LL}};
  time_grid = SteadyTimeGrid{start, interval};
  EXPECT_TRUE(time_grid.is_initialized());
}

/// \test Check that the next timestamp is correctly predicted.
TEST(SteadyTimeGridTest, GetNextTimestamp) {
  const std::chrono::system_clock::time_point start{std::chrono::seconds{42LL}};
  const std::chrono::system_clock::duration interval{std::chrono::milliseconds{50LL}};
  SteadyTimeGrid time_grid{start, interval};

  const auto timestamp_next_to_start = start + interval;
  const auto predicted_from_start = time_grid.get_next_timestamp_after(start);
  EXPECT_EQ(timestamp_next_to_start, predicted_from_start);

  const std::chrono::system_clock::time_point timestamp{start + std::chrono::milliseconds{42LL}};
  const auto predicted_from_after_start = time_grid.get_next_timestamp_after(timestamp);
  EXPECT_EQ(timestamp_next_to_start, predicted_from_after_start);
}
