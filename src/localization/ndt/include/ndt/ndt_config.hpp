// Copyright 2020 Apex.AI, Inc.
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

#ifndef NDT__NDT_CONFIG_HPP_
#define NDT__NDT_CONFIG_HPP_

#include <ndt/ndt_common.hpp>
#include <voxel_grid/config.hpp>
#include <utility>

namespace autoware
{
namespace localization
{
namespace ndt
{
/// Base config class for all ndt localizers
/// \tparam OptimizerOptionsT Configuration class type for the utilized optimizer.
/// \tparam OptimizationConfigT Configuration class type for the used optimization problem.
template<typename OptimizerOptionsT, typename OptimizationConfigT>
class NDTLocalizerConfigBase
{
public:
  using MapConfig = perception::filters::voxel_grid::Config;
  using OptimizerOptions = OptimizerOptionsT;

  /// Constructor
  /// \param optimization_config rvalue of the optimization config.
  /// \param optimizer_options
  /// \param map_config
  /// \param guess_time_tolerance
  NDTLocalizerConfigBase(
    OptimizationConfigT && optimization_config,
    OptimizerOptionsT && optimizer_options,
    MapConfig && map_config,
    std::chrono::nanoseconds guess_time_tolerance)
  : m_optimization_config{std::forward(optimization_config)},
    m_optimizer_options{std::forward(optimizer_options)},
    m_map_config{std::forward<MapConfig>(map_config)},
    m_guess_time_tol{guess_time_tolerance}
  {}

  /// Constructor
  /// \param optimization_config Optimization configuration
  /// \param optimizer_options Optimizer configuration
  /// \param map_config Map configuration
  /// \param guess_time_tolerance Time difference tolerance between the initial guess timestamp
  /// and the timestamp of the scan.
  NDTLocalizerConfigBase(
    const OptimizationConfigT & optimization_config,
    const OptimizerOptionsT & optimizer_options,
    const MapConfig & map_config,
    std::chrono::nanoseconds guess_time_tolerance)
  : m_optimization_config{optimization_config},
    m_optimizer_options{optimizer_options},
    m_map_config{map_config},
    m_guess_time_tol{guess_time_tolerance}
  {}

  /// Get optimization config.
  /// \return optimization config
  const OptimizationConfigT & optimization_config() const noexcept
  {
    return m_optimization_config;
  }

  /// Get optimizer config.
  /// \return optimizer config
  const OptimizerOptionsT & optimizer_options() const noexcept
  {
    return m_optimizer_options;
  }

  // TODO(yunus.caliskan): Consider delegating map config get/set to the implementation
  // when map types with different configuration needs are added.
  /// Get map config.
  /// \return Map config
  const MapConfig & map_config() const noexcept
  {
    return m_map_config;
  }

  /// Get optimizer config.
  /// \return optimizer config
  const std::chrono::nanoseconds & guess_time_tolerance() const noexcept
  {
    return m_guess_time_tol;
  }

private:
  OptimizationConfigT m_optimization_config;
  OptimizerOptionsT m_optimizer_options;
  MapConfig m_map_config;
  std::chrono::nanoseconds m_guess_time_tol;
};


/// Config class for p2d optimziation problem
class NDT_PUBLIC P2DNDTOptimizationConfig
{
public:
  /// Constructor
  /// \param outlier_ratio Outlier ratio to be used in the gaussian distribution variation used
  /// in (eq. 6.7) [Magnusson 2009]
  explicit P2DNDTOptimizationConfig(Real outlier_ratio);

  /// Get outlier ratio.
  /// \return outlier ratio.
  Real outlier_ratio() const noexcept;

private:
  Real m_outlier_ratio;
};

/// config class for p2d ndt localizer
/// \tparam OptimizerOptionsT Configuration class type for the utilized optimizer.
template<typename OptimizerOptionsT>
class NDT_PUBLIC P2DNDTLocalizerConfig
  : public NDTLocalizerConfigBase<OptimizerOptionsT, P2DNDTOptimizationConfig>
{
public:
  using MapConfig = perception::filters::voxel_grid::Config;

  /// Constructor
  /// \param optimization_config Optimization configuration
  /// \param optimizer_options Optimizer configuration
  /// \param map_config Map configuration
  /// \param scan_capacity Capacity of the ndt scan. This corresponds to the maximum number of
  /// points expected in a single lidar scan.
  /// \param guess_time_tolerance Time difference tolerance between the initial guess timestamp
  /// and the timestamp of the scan.
  P2DNDTLocalizerConfig(
    const P2DNDTOptimizationConfig & optimization_config,
    const OptimizerOptionsT & optimizer_options,
    const MapConfig & map_config,
    const uint32_t scan_capacity,
    std::chrono::nanoseconds guess_time_tolerance)
  : NDTLocalizerConfigBase<OptimizerOptionsT, P2DNDTOptimizationConfig>
      (optimization_config, optimizer_options, map_config, guess_time_tolerance),
    m_scan_capacity(scan_capacity) {}


  /// Get scan capacity.
  /// \return scan capacity.
  uint32_t scan_capacity() const noexcept
  {
    return m_scan_capacity;
  }

private:
  uint32_t m_scan_capacity;
};

}  // namespace ndt
}  // namespace localization
}  // namespace autoware

#endif  // NDT__NDT_CONFIG_HPP_
