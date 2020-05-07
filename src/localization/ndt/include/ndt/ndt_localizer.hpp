// Copyright 2019 Apex.AI, Inc.
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

#ifndef NDT__NDT_LOCALIZER_HPP_
#define NDT__NDT_LOCALIZER_HPP_

#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <ndt/ndt_common.hpp>
#include <ndt/ndt_optimization_problem.hpp>
#include <optimization/optimizer_options.hpp>
#include <utility>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{
using CloudT = sensor_msgs::msg::PointCloud2;

/// Base class for NDT based localizers. Implementations must implement the validation logic.
/// \tparam ScanT Type of ndt scan.
/// \tparam MapT Type of ndt map.
/// \tparam NDTOptimizationProblemT Type of ndt optimization problem.
/// \tparam OptimizerT Type of optimizer.
/// \tparam ConfigT Type of localization configuration.
template<typename ScanT, typename MapT, typename NDTOptimizationProblemT,
  typename OptimizerT, typename ConfigT>
class NDT_PUBLIC NDTLocalizerBase : public localization_common::RelativeLocalizerBase<CloudT,
    CloudT, localization_common::OptimizedRegistrationSummary>
{
public:
  using Transform = geometry_msgs::msg::TransformStamped;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using OptimizerOptions = typename ConfigT::OptimizerOptions;
  using Base = localization_common::RelativeLocalizerBase<CloudT,
      CloudT, localization_common::OptimizedRegistrationSummary>;
  /// Constructor
  /// \param config NDT localizer config
  /// \param scan Initial value of the ndt scan. This element is expected to be constructed in the
  /// implementation class and moved to the base class.
  /// \param map Initial value of the ndt map. This element is expected to be constructed in the
  /// implementation class and moved to the base class.
  /// \param optimizer Optimizer to use during optimization.
  explicit NDTLocalizerBase(
    const ConfigT & config, ScanT && scan, MapT && map,
    const OptimizerT & optimizer)
  : m_optimizer{optimizer}, m_config(config),
    m_scan{std::forward<ScanT>(scan)}, m_map{std::forward<MapT>(map)},
    m_optimizer_options{config.optimizer_options()} {}

  /// Register a measurement to the current map and return the transformation from map to the
  /// measurement.
  /// \param[in] msg Point cloud to be registered.
  /// \param[in] transform_initial Initial transformation guess.
  /// \param[out] pose_out Transformation from the map frame to the measurement's frame.
  /// \return Registration summary. T
  /// \throws std::logic_error on measurements older than the map.
  /// \throws std::domain_error on pose estimates that are not within the configured duration
  /// range from the measurement.
  /// \throws std::runtime_error on numerical errors in the optimizer.
  RegistrationSummary register_measurement_impl(
    const CloudT & msg,
    const Transform & transform_initial, PoseWithCovarianceStamped & pose_out) override
  {
    validate_msg(msg);
    validate_guess(msg, transform_initial);
    // Initial checks passed, proceed with initialization
    // Eigen representations to be used for internal computations.
    EigenPose<Real> eig_pose_initial, eig_pose_result;
    eig_pose_initial.setZero();
    eig_pose_result.setZero();
    // Convert the ros transform/pose to eigen pose vector
    transform_adapters::transform_to_pose(transform_initial.transform, eig_pose_initial);

    // Set the scan
    m_scan.clear();
    m_scan.insert(msg);

    // Define and solve the problem.
    NDTOptimizationProblemT problem(m_scan, m_map, m_config.optimization_config());
    const auto opt_summary = m_optimizer.solve(problem, eig_pose_initial, eig_pose_result,
        m_optimizer_options);

    if (opt_summary.termination_type() == common::optimization::TerminationType::FAILURE) {
      throw std::runtime_error("NDT localizer has likely encountered a numerical "
              "error during optimization.");
    }

    // Convert eigen pose back to ros pose/transform
    transform_adapters::pose_to_transform(eig_pose_result,
      pose_out.pose.pose);

    pose_out.header.stamp = msg.header.stamp;
    pose_out.header.frame_id = map_frame_id();

    // Populate covariance information. It is implementation defined.
    set_covariance(problem, eig_pose_initial, eig_pose_result, pose_out);
    return localization_common::OptimizedRegistrationSummary{opt_summary};
  }

  /// Replace the map with a given message
  /// \param msg Message containing the map
  void set_map_impl(const CloudT & msg) override
  {
    m_map.clear();
    m_map.insert(msg);
  }

  /// Insert the given message to the existing map.
  /// \param msg Message containing the map addition.
  void insert_to_map_impl(const CloudT & msg) override
  {
    m_map.insert(msg);
  }

  /// Get the last used scan.
  const ScanT & scan() const noexcept
  {
    return m_scan;
  }
  /// Get the current map.
  const MapT & map() const noexcept
  {
    return m_map;
  }
  /// Get the optimizer.
  const OptimizerT & optimizer() const noexcept
  {
    return m_optimizer;
  }
  /// Get the localizer configuration.
  const ConfigT & config() const noexcept
  {
    return m_config;
  }

  /// Get the frame id of the current map.(Required for base interface)
  const std::string & map_frame_id() const noexcept override
  {
    return m_map.frame_id();
  }

  /// Get the timestamp of the current map. (Required for base interface)
  std::chrono::system_clock::time_point map_stamp() const noexcept override
  {
    return m_map.stamp();
  }


  virtual ~NDTLocalizerBase() = default;

protected:
  /// Populate the covariance information of an ndt estimate using the information using existing
  /// information regarding scan, map and the optimization problem.
  /// \param[in] problem Optimization problem.
  /// \param[in] initial_guess Initial transformation guess as a pose.
  /// \param[in] pose_result Estimated transformation as a pose.
  /// \param[out] solution Estimated transform message.
  virtual void set_covariance(
    const NDTOptimizationProblemT & problem,
    const EigenPose<Real> & initial_guess,
    const EigenPose<Real> & pose_result,
    PoseWithCovarianceStamped & solution) const
  {
    (void) problem;
    (void) initial_guess;
    (void) pose_result;
    (void) solution;
    // For now, do nothing.
  }

  /// Check if the received message is valid to be registered. Following checks are made:
  /// * Message timestamp is not older than the map timestamp.
  /// \param msg Message to register.
  /// \throws std::logic_error on old data.
  virtual void validate_msg(const CloudT & msg) const
  {
    const auto message_time = ::time_utils::from_message(msg.header.stamp);
    // Map shouldn't be newer than a measurement.
    if (message_time < m_map.stamp()) {
      throw std::logic_error("Lidar scan should not have a timestamp older than the timestamp of"
              "the current map.");
    }
  }

  /// Check if the initial guess is valid. Following checks are made:
  /// * pose guess timestamp is within a tolerated range from the scan timestamp.
  /// \param msg Message to register
  /// \param transform_initial Initial pose estimate
  /// \throws std::domain_error on untimely initial pose.
  virtual void validate_guess(const CloudT & msg, const Transform & transform_initial) const
  {
    const auto message_time = ::time_utils::from_message(msg.header.stamp);

    const auto guess_scan_diff = ::time_utils::from_message(transform_initial.header.stamp) -
      message_time;
    const auto stamp_tol = m_config.guess_time_tolerance();
    // An initial estimate should be comparable in time to the measurement's time stamp
    if (std::abs(std::chrono::duration_cast<std::decay_t<decltype(stamp_tol)>>(guess_scan_diff).
      count()) >
      std::abs(stamp_tol.count()))
    {
      throw std::domain_error("Initial guess is not within: " + std::to_string(stamp_tol.count()) +
              "ns range of the scan's time stamp. Either increase the tolerance range or"
              "make sure the localizer takes in timely initial pose guesses.");
    }
  }

private:
  OptimizerT m_optimizer;
  ConfigT m_config;
  ScanT m_scan;
  MapT m_map;
  OptimizerOptions m_optimizer_options;
};

/// P2D localizer implementation.
/// This implementation specifies a covariance computation method for the P2D optimization problem.
/// \tparam OptimizerT Optimizer type.
/// \tparam OptimizerOptionsT Optimizer options type.
template<typename OptimizerT, typename OptimizerOptionsT>
class NDT_PUBLIC P2DNDTLocalizer : public NDTLocalizerBase<P2DNDTScan, StaticNDTMap,
    P2DNDTOptimizationProblem, OptimizerT, P2DNDTLocalizerConfig<OptimizerOptionsT>>
{
public:
  using CloudT = sensor_msgs::msg::PointCloud2;
  using ParentT = NDTLocalizerBase<P2DNDTScan, StaticNDTMap,
      P2DNDTOptimizationProblem, OptimizerT, P2DNDTLocalizerConfig<OptimizerOptionsT>>;
  using Base = typename ParentT::Base;
  using Transform = typename ParentT::Transform;
  using PoseWithCovarianceStamped = typename ParentT::PoseWithCovarianceStamped;
  using ConfigT = P2DNDTLocalizerConfig<OptimizerOptionsT>;
  using ScanT = P2DNDTScan;
  using MapT = StaticNDTMap;

  explicit P2DNDTLocalizer(const ConfigT & config, const OptimizerT & optimizer)
  : ParentT(config, ScanT{config.scan_capacity()}, MapT{config.map_config()}, optimizer) {}

protected:
  void set_covariance(
    const P2DNDTOptimizationProblem &,
    const EigenPose<Real> &,
    const EigenPose<Real> &,
    PoseWithCovarianceStamped &) const override
  {
    // For now, do nothing.
  }
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_LOCALIZER_HPP_
