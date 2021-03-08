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

#ifndef STATE_ESTIMATION_NODES__HISTORY_HPP_
#define STATE_ESTIMATION_NODES__HISTORY_HPP_

#include <common/types.hpp>
#include <mpark_variant_vendor/variant.hpp>
#include <state_estimation_nodes/steady_time_grid.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <utility>

namespace autoware
{
namespace prediction
{

namespace detail
{

/// Check if the sample passes the Mahalanobis gate.
///
/// @param[in]  sample                 The input sample
/// @param[in]  mean                   The mean to be compared against
/// @param[in]  covariance_factor      Covariance around the given mean
/// @param[in]  mahalanobis_threshold  The mahalanobis threshold factor
///
/// @tparam     kNumOfStates           Number of states in the state vector.
///
/// @return     True if the sample passes the gate and false otherwise.
///
template<std::int32_t kNumOfStates>
bool passes_mahalanobis_gate(
  const Eigen::Matrix<common::types::float32_t, kNumOfStates, 1> & sample,
  const Eigen::Matrix<common::types::float32_t, kNumOfStates, 1> & mean,
  const Eigen::Matrix<common::types::float32_t, kNumOfStates, kNumOfStates> & covariance_factor,
  const common::types::float32_t mahalanobis_threshold)
{
  // This is equivalent to the squared Mahalanobis distance of the form: diff.T * C.inv() * diff
  // Instead of the covariance matrix C we have its lower-triangular factor L, such that C = L * L.T
  // squared_mahalanobis_distance = diff.T * C.inv() * diff
  // = diff.T * (L * L.T).inv() * diff
  // = diff.T * L.T.inv() * L.inv() * diff
  // = (L.inv() * diff).T * (L.inv() * diff)
  // this allows us to efficiently find the squared Mahalanobis distance using (L.inv() * diff),
  // which can be found as a solution to: L * x = diff.
  const auto diff = sample - mean;
  const auto squared_threshold = mahalanobis_threshold * mahalanobis_threshold;
  const auto x = covariance_factor.ldlt().solve(diff);
  return (x.transpose() * x) < squared_threshold;
}

}  // namespace detail

/// @brief      An event to indicate a prediction step.
struct PredictionEvent {};

///
/// @brief      An event to reset the state of the filter.
///
/// @tparam     FilterT  Type of the EKF filter used to infer vector and matrix types.
///
template<typename FilterT>
struct ResetEvent
{
  typename FilterT::state_vec_t state;
  typename FilterT::square_mat_t covariance_factor;
};

///
/// @brief      This class encapsulates a history of events used with EKF.
///
///             The class handles adding events to a history of a specified size. It models the
///             behavior of a circular buffer, meaning that as new events come in, the oldest ones
///             are removed. The events can be either measurement types or specific events like
///             reset or prediction. Whenever an event is added to the middle of the history all the
///             following events get rolled on top of this event to produce a new state.
///
/// @tparam     FilterT       Type of EKF filter used.
/// @tparam     kNumOfStates  Dimensionality of the state in the filter.
/// @tparam     EventT        A variadic template of all possible events.
///
template<
  typename FilterT,
  std::int32_t kNumOfStates,
  typename ... EventT>
class History
{
  ///
  /// @brief      A single entry in the history. Holds state and covariance and a variant of events.
  ///
  class HistoryEntry;
  ///
  /// @brief      A functor that handles upading the state. It is used with the variant stored
  ///             within the HistoryEntry.
  ///
  class EkfStateUpdater;

  /// Typedef for timestamps.
  using Timestamp = std::chrono::system_clock::time_point;
  /// Typedef for history map type.
  using HistoryMap = std::multimap<Timestamp, HistoryEntry>;

public:
  ///
  /// @brief      Construct history from a filter pointer with a specific size.
  ///
  /// @param      filter                 The filter pointer to be used internally.
  /// @param[in]  max_history_size       The maximum history size.
  /// @param[in]  mahalanobis_threshold  The mahalanobis threshold
  ///
  explicit History(
    FilterT & filter,
    const std::size_t max_history_size,
    const common::types::float32_t mahalanobis_threshold)
  : m_filter{filter},
    m_max_history_size{max_history_size},
    m_mahalanobis_threshold{mahalanobis_threshold} {}

  ///
  /// @brief      Add an event to history. If it is added to the middle the following ones are
  ///             automatically replayed on top of it.
  ///
  /// @param[in]  timestamp  The timestamp of the event.
  /// @param[in]  entry      The entry to be added to history.
  ///
  void emplace_event(const Timestamp & timestamp, const HistoryEntry & entry);
  /// @brief      Check if the history is empty.
  inline bool empty() const noexcept {return m_history.empty();}
  /// @brief      Get size of history.
  inline std::size_t size() const noexcept {return m_history.size();}
  /// @brief      Get last timestamp in history.
  inline const Timestamp & get_last_timestamp() const noexcept {return m_history.rbegin()->first;}
  /// @brief      Get last event in history.
  inline const HistoryEntry & get_last_event() const noexcept {return m_history.rbegin()->second;}
  /// @brief      Get the filter as a const ref.
  const FilterT & get_filter() const noexcept {return m_filter;}
  /// @brief      Get the filter.
  FilterT & get_filter() noexcept {return m_filter;}

private:
  ///
  /// @brief      If the history is too large, drop the oldest event from it.
  ///
  inline void drop_oldest_event_if_needed()
  {
    if ((m_history.size() >= m_max_history_size) && (m_max_history_size > 0U)) {
      (void) m_history.erase(m_history.begin());
    }
  }

  ///
  /// @brief      Update all the following events as their state is based on the current one.
  ///
  /// @param[in]  start_iter  The current iterator with the new state.
  ///
  void update_impacted_events(const typename HistoryMap::iterator & start_iter);

  HistoryMap m_history{};  ///< history of events.
  FilterT & m_filter{};  ///< pointer to the filter implementation.
  std::size_t m_max_history_size{};  ///< Maximum number of events in history.
  common::types::float32_t m_mahalanobis_threshold{};  ///< Mahalanobis distance threshold.
};

template<typename FilterT, std::int32_t kNumOfStates, typename ... EventT>
class History<FilterT, kNumOfStates, EventT...>::HistoryEntry
{
public:
  template<typename SingleEventT>
  // cppcheck-suppress noExplicitConstructor; Conversion to the variant type takes place.
  HistoryEntry(const SingleEventT & event) : m_event {event} {}

  /// @brief      Update the stored state.
  void update_stored_state(const typename FilterT::state_vec_t & state) noexcept
  {
    m_stored_state = state;
  }
  /// @brief      Get the stored state.
  const typename FilterT::state_vec_t & stored_state() const noexcept {return m_stored_state;}
  /// @brief      Set the covariance left factor to be stored there.
  void update_stored_covariance_factor(const typename FilterT::square_mat_t & factor) noexcept
  {
    m_stored_covariance_factor = factor;
  }
  /// @brief      Get the stored covariance in the form of its left factor.
  const typename FilterT::square_mat_t & stored_covariance_factor() const noexcept
  {
    return m_stored_covariance_factor;
  }
  /// @brief      Get the event stored in this history entry.
  const mpark::variant<EventT...> & event() const {return m_event;}

private:
  /// State stored in this entry.
  typename FilterT::state_vec_t m_stored_state{FilterT::state_vec_t::Zero()};
  /// Left covariance factor stored in this entry.
  typename FilterT::square_mat_t m_stored_covariance_factor{FilterT::square_mat_t::Zero()};
  /// Event stored in this history entry.
  mpark::variant<EventT...> m_event;
};

template<typename FilterT, std::int32_t kNumOfStates, typename ... EventT>
class History<FilterT, kNumOfStates, EventT...>::EkfStateUpdater
{
public:
  explicit EkfStateUpdater(
    FilterT & filter,
    const common::types::float32_t mahalanobis_threshold,
    const std::chrono::system_clock::duration & dt = std::chrono::milliseconds{0})
    : m_filter {filter}, m_mahalanobis_threshold{mahalanobis_threshold}, m_dt{dt}
  {}

  ///
  /// @brief      An operator that passes a measurement event to the filter implementation.
  ///
  /// @param[in]  event         A measurement event.
  ///
  /// @tparam     MeasurementT  Type of measurement event. Can be any measurement type.
  ///
  template<typename MeasurementT>
  void operator()(const MeasurementT & event)
  {
    m_filter.temporal_update(m_dt);
    // TODO(#887): I see a couple of ways to check mahalanobis distance in case the measurement does
    // not cover the full state. Here I upscale it to the full state, copying the values of the
    // current state for ones missing in the observation. We can alternatively apply the H matrix
    // and only compute the distance in the measurement world. Don't really know which one is best
    // here.
    if (!detail::passes_mahalanobis_gate(
        event.get_values_in_full_state(m_filter.get_state()),
        m_filter.get_state(),
        m_filter.get_covariance(),
        m_mahalanobis_threshold)) {return;}
    (void) m_filter.observation_update(
      event.get_values(),
      MeasurementT::template get_observation_to_state_mapping<kNumOfStates>(),
      event.get_variances());
  }

  /// @brief      An operator that resets the state of the filter implementation.
  void operator()(const ResetEvent<FilterT> & event)
  {
    m_filter.reset(event.state, event.covariance_factor);
  }

  /// @brief      An operator that applies the prediction event to the filter implementation.
  void operator()(const PredictionEvent &)
  {
    m_filter.temporal_update(m_dt);
  }

private:
  FilterT & m_filter{};  ///< A pointer to the filter implementation.
  common::types::float32_t m_mahalanobis_threshold{};  ///< Mahalanobis distance threshold.
  std::chrono::system_clock::duration m_dt{};  ///< Current time step.
};

template<typename FilterT, std::int32_t kNumOfStates, typename ... EventT>
void History<FilterT, kNumOfStates, EventT...>::emplace_event(
  const Timestamp & timestamp, const HistoryEntry & entry)
{
  drop_oldest_event_if_needed();
  const auto iterator_to_inserted_position = m_history.emplace(timestamp, entry);
  update_impacted_events(iterator_to_inserted_position);
}

template<typename FilterT, std::int32_t kNumOfStates, typename ... EventT>
void History<FilterT, kNumOfStates, EventT...>::update_impacted_events(
  const typename HistoryMap::iterator & start_iter)
{
  Timestamp previous_timestamp{};
  if (start_iter == m_history.begin()) {
    if (!mpark::holds_alternative<ResetEvent<FilterT>>(start_iter->second.event())) {
      (void) m_history.erase(start_iter);
      throw std::runtime_error(
              "Non-reset event inserted to the beginning of history. This might "
              "happen if a very old event is inserted into the queue. Consider "
              "increasing the queue size or debug program latencies.");
    }
  } else {
    const auto prev_iter = std::prev(start_iter);
    previous_timestamp = prev_iter->first;
    const auto & prev_entry = prev_iter->second;
    m_filter.reset(prev_entry.stored_state(), prev_entry.stored_covariance_factor());
  }
  for (auto iter = start_iter; iter != m_history.end(); ++iter) {
    const auto current_timestamp = iter->first;
    auto & entry = iter->second;
    mpark::visit(
      EkfStateUpdater{m_filter, m_mahalanobis_threshold, current_timestamp - previous_timestamp},
      entry.event());
    entry.update_stored_state(m_filter.get_state());
    entry.update_stored_covariance_factor(m_filter.get_covariance());
    previous_timestamp = iter->first;
  }
}


}  // namespace prediction
}  // namespace autoware


#endif  // STATE_ESTIMATION_NODES__HISTORY_HPP_
