// Copyright 2017-2019 the Autoware Foundation
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

#include <gtest/gtest.h>
#include <vector>
#include "autoware_auto_algorithm/algorithm.hpp"

template<typename Container, typename RandomIt = typename Container::iterator>
using QuickSorter = ::autoware::common::algorithm::QuickSorter<Container, RandomIt>;

TEST(QuickSortIterative, Empty) {
  ::std::vector<int32_t> vector;
  QuickSorter<::std::vector<int32_t>> sorter;
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int32_t>({}));
  ASSERT_EQ(sorter.capacity(), 0UL);
}

TEST(QuickSortIterative, SingleElem) {
  ::std::vector<int32_t> vector = {42};
  QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 1UL);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int32_t>({42}));
  ASSERT_EQ(sorter.capacity(), 1UL);
}

TEST(QuickSortIterative, TwoElems) {
  ::std::vector<int32_t> vector = {42, 43};
  QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 2UL);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int32_t>({42, 43}));
  ASSERT_EQ(sorter.capacity(), 2UL);
}

TEST(QuickSortIterative, AlreadySorted) {
  ::std::vector<int32_t> vector = {1, 2, 3, 4, 5, 6};
  QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6UL);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int32_t>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(QuickSortIterative, Descending) {
  ::std::vector<int32_t> vector = {6, 5, 4, 3, 2, 1};
  QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6UL);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int32_t>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(QuickSortIterative, Random) {
  ::std::vector<int32_t> vector = {3, 5, 1, 6, 4, 2};
  QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6UL);
  sorter.sort(vector.begin(), vector.end());
  ASSERT_EQ(vector, ::std::vector<int32_t>({1, 2, 3, 4, 5, 6}));
  ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(QuickSortIterative, SubRangeBegin) {
  ::std::vector<int32_t> vector = {3, 5, 1, 6, 4, 2};
  QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6UL);
  sorter.sort(vector.begin(), vector.end() - 2);
  ASSERT_EQ(vector, ::std::vector<int32_t>({1, 3, 5, 6, 4, 2}));
  ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(QuickSortIterative, SubRangeEnd) {
  ::std::vector<int32_t> vector = {3, 5, 1, 6, 4, 2};
  QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6UL);
  sorter.sort(vector.begin() + 2, vector.end());
  ASSERT_EQ(vector, ::std::vector<int32_t>({3, 5, 1, 2, 4, 6}));
  ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(QuickSortIterative, SubRange) {
  ::std::vector<int32_t> vector = {3, 5, 1, 6, 4, 2};
  QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
  ASSERT_EQ(sorter.capacity(), 6UL);
  sorter.sort(vector.begin() + 1, vector.end() - 3);
  ASSERT_EQ(vector, ::std::vector<int32_t>({3, 1, 5, 6, 4, 2}));
  ASSERT_EQ(sorter.capacity(), 6UL);
}
