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
/// \brief This file defines the RadiusSearch2DFilterNode class.

#ifndef OUTLIER_FILTER_NODES__RADIUS_SEARCH_2D_FILTER_NODE_HPP_
#define OUTLIER_FILTER_NODES__RADIUS_SEARCH_2D_FILTER_NODE_HPP_

#include <vector>
#include <memory>

#include "common/types.hpp"
#include "outlier_filter_nodes/visibility_control.hpp"

#include "filter_node_base/filter_node_base.hpp"
#include "outlier_filter/radius_search_2d_filter.hpp"

#include "rclcpp/rclcpp.hpp"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

/// \class RadiusSearch2DFilterNode
/// \brief Node inheriting from FilterNodeBase and is the ROS2 node interface for the
//   RadiusSearch2DFilter library
class OUTLIER_FILTER_NODES_PUBLIC RadiusSearch2DFilterNode final : public filter_node_base::
  FilterNodeBase
{
public:
  /** \brief The default constructor for the RadiusSearch2DFilterNode class
   * \param options An rclcpp::NodeOptions object to pass on to the FilterNodeBase class
   */
  explicit RadiusSearch2DFilterNode(const rclcpp::NodeOptions & options);

protected:
  /** \brief Implementation of the FilterNodeBase class abstract filter method
   *
   * Converts the point cloud into the PCL::PointCloud<pcl::PointXYZ> type to be processed by the
   * RadiusSearch2DFilter library. The method then returns the filter point cloud via the output
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
  /** \brief Class object containing the RadiusSearch2DFilter library functionality */
  std::shared_ptr<perception::filters::outlier_filter::radius_search_2d_filter::
    RadiusSearch2DFilter> radius_search_2d_filter_;

  /** \brief Variable containing the value of the search radius (passed into
       radius_search_2d_filter_) */
  common::types::float64_t search_radius_;

  /** \brief Variable containing the value of the minimum neighbors for points (passed into
     radius_search_2d_filter_) */
  std::int64_t min_neighbors_;
};
}  // namespace outlier_filter_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // OUTLIER_FILTER_NODES__RADIUS_SEARCH_2D_FILTER_NODE_HPP_
