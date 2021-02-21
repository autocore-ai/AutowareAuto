// Copyright 2019 Silexica GmbH, Lichtstr. 25, Cologne, Germany. All rights reserved.
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
/// \file
/// \brief This file provides an iterative quick sort implementation.
#ifndef AUTOWARE_AUTO_ALGORITHM__QUICK_SORT_HPP_
#define AUTOWARE_AUTO_ALGORITHM__QUICK_SORT_HPP_

#include <algorithm>
#include <functional>
#include <utility>
#include <vector>

namespace autoware
{

namespace common
{

namespace algorithm
{

/// \brief Iterative quick sort implementation based on a stack.
template<typename Container, typename RandomIt = typename Container::iterator>
class QuickSorter
{
public:
  QuickSorter(QuickSorter const &) = delete;
  QuickSorter & operator=(QuickSorter const &) = delete;
  QuickSorter(QuickSorter &&) = default;
  /// \brief Move equals operator
  QuickSorter & operator=(QuickSorter &&) = default;

  /// \brief Default constructor, do not reserve capacity for stack
  QuickSorter() = default;

  /// \brief Construct and reserve capacity for stack
  /// \param[in] capacity - The maximum capacity of the container to be sorted
  explicit QuickSorter(::std::size_t capacity)
  {
    reserve(capacity);
  }

  /// \brief Iterative quick sort implementation using a stack, sorts
  /// range [first, last) using the given comparison function
  /// \param[in] first Start of the range to sort
  /// \param[in] last End of the range to sort (not included)
  /// \param[in] comp The comparison function to base the sorting on
  template<typename Compare>
  void sort(RandomIt first, RandomIt last, Compare comp) const
  {
    if (::std::distance(first, last) < 2) {
      return;
    }

    // Make sure we do not accidently have an already partially filled stack,
    // capacity does not change
    m_stack.clear();

    // Add first interval to the stack for sorting, from here on last is really
    // the last element and not the element after, i.e. not end
    m_stack.push_back(first);
    m_stack.push_back(last - 1);

    while (!m_stack.empty()) {
      last = m_stack.back();
      m_stack.pop_back();
      first = m_stack.back();
      m_stack.pop_back();

      auto part = QuickSorter::partition(first, last, comp);

      if (part > first + 1) {
        m_stack.push_back(first);
        m_stack.push_back(part - 1);
      }

      if (part < last - 1) {
        m_stack.push_back(part + 1);
        m_stack.push_back(last);
      }
    }
  }

  /// \brief Iterative quick sort implementation using a stack, sorts
  /// range [first, last) using the default less operation
  /// \param[in] first Start of the range to sort
  /// \param[in] last End of the range to sort (not included)
  void sort(RandomIt first, RandomIt last) const
  {
    sort(first, last, ::std::less<const decltype(*first)>());
  }

  /// \brief Reserves helper stack capacity for the iterative quick sort
  /// algorithm based on the capacity of the container to be sorted such that
  /// no heap allocation is done during the algorithm.
  /// \param[in] capacity - The maximum capacity of the container to be sorted
  void reserve(::std::size_t capacity)
  {
    // The maximum partition depth is n/2 + 1, which means we need a maximum
    // capacity of n + 2 to hold store the iterators in the stack.
    m_stack.reserve(capacity + 2);
  }

  /// \brief Returns the maximum capacity that is allowed for a container to be
  /// sorted.
  /// \return The maximum capacity that a container may have if it is to be sorted
  /// using this sorter.
  ::std::size_t capacity() const
  {
    if (m_stack.capacity() < 2) {
      return 0;
    }
    return m_stack.capacity() - 2;
  }

private:
  /// \brief Partition range [first, last], based on pivot element last. After
  /// execution all elements smaller than the pivot element are left of it and
  /// all bigger elements right of it.
  /// \param[in] first Start of the range to partition
  /// \param[in] last End (included) of the range to partition, used as pivot
  /// \param[in] comp Element comparison function
  /// \return Iterator to the pivot element in the range
  template<typename Compare>
  static RandomIt partition(RandomIt first, RandomIt last, Compare comp)
  {
    auto prev = first;

    // Iterate over range and swap whenever element is smaller than the pivot
    // element.
    for (auto it = first; it < last; it++) {
      if (comp(*it, *last)) {
        ::std::iter_swap(it, prev);
        prev++;
      }
    }

    // Swap the pivot element into place
    ::std::iter_swap(prev, last);
    return prev;
  }

private:
  /// Helper stack used for sorting, needs to have capacity (last-first)+2
  mutable ::std::vector<RandomIt> m_stack;
};

}  // namespace algorithm

}  // namespace common

}  // namespace autoware

#endif  // AUTOWARE_AUTO_ALGORITHM__QUICK_SORT_HPP_
