// Copyright 2021 Tier IV, Inc
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

/// \copyright Copyright 2021 Tier IV, Inc
/// \file
/// \brief This file defines the VoxelGridOutlierFilterNode class.

#ifndef OUTLIER_FILTER_NODES__VOXEL_GRID_OUTLIER_FILTER_NODE_HPP_
#define OUTLIER_FILTER_NODES__VOXEL_GRID_OUTLIER_FILTER_NODE_HPP_

#include <memory>
#include <vector>

#include "common/types.hpp"
#include "outlier_filter_nodes/visibility_control.hpp"

#include "filter_node_base/filter_node_base.hpp"
#include "outlier_filter/voxel_grid_outlier_filter.hpp"
#include "rclcpp/rclcpp.hpp"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

/// \class VoxelGridOutlierFilterNode
/// \brief Node inheriting from FilterNodeBase and is the ROS2 node interface for the
//   VoxelGridOutlierFilter library
class OUTLIER_FILTER_NODES_PUBLIC VoxelGridOutlierFilterNode final : public filter_node_base::
  FilterNodeBase
{
public:
  /** \brief The default constructor for the VoxelGridOutlierFilterNode class
   * \param options An rclcpp::NodeOptions object to pass on to the FilterNodeBase class
   */
  explicit VoxelGridOutlierFilterNode(const rclcpp::NodeOptions & options);

protected:
  /** \brief Implementation of the FilterNodeBase class abstract filter method
   *
   * Converts the point cloud into the PCL::PointCloud<pcl::PointXYZ> type to be processed by the
   * VoxelGridOutlierFilter library. The method then returns the filter point cloud via the output
   * argument.
   *
   * \param input The input point cloud dataset.
   * \param output The resultant filtered PointCloud2
   */
  OUTLIER_FILTER_NODES_PUBLIC void filter(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output) override;

  /** \brief Implementation of the FilterNodeBase class abstract get_node_parameters method
   *
   * This method is called when a parameter is set. From the resulting parameters it will get all of
   * the associated parameters and update the filter library class with the new values. The result
   * of retrieving the parameter is returned.
   *
   * \param p Vector of rclcpp::Parameters belonging to the node
   * \return rcl_interfaces::msg::SetParametersResult Result of retrieving the parameter
   */
  OUTLIER_FILTER_NODES_PUBLIC rcl_interfaces::msg::SetParametersResult get_node_parameters(
    const std::vector<rclcpp::Parameter> & p) override;

private:
  /** \brief Class object containing the VoxelGridOutlierFilter library functionality */
  std::shared_ptr<perception::filters::outlier_filter::voxel_grid_outlier_filter::
    VoxelGridOutlierFilter> voxel_grid_outlier_filter_;

  /** \brief Variable containing the value of the voxel x dimension (passed into
     voxel_grid_outlier_filter_) */
  common::types::float64_t voxel_size_x_;

  /** \brief Variable containing the value of the voxel y dimension (passed into
     voxel_grid_outlier_filter_) */
  common::types::float64_t voxel_size_y_;

  /** \brief Variable containing the value of the voxel z dimension (passed into
     voxel_grid_outlier_filter_) */
  common::types::float64_t voxel_size_z_;

  /** \brief Variable containing the points threshold (passed into voxel_grid_outlier_filter_) */
  std::int64_t voxel_points_threshold_;
};
}  // namespace outlier_filter_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // OUTLIER_FILTER_NODES__VOXEL_GRID_OUTLIER_FILTER_NODE_HPP_
