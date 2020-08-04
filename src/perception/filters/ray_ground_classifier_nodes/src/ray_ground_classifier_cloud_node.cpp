// Copyright 2018-2020 Apex.AI, Inc., Arm Limited
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
#include <common/types.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdlib.h>
#include <string>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier_nodes
{
////////////////////////////////////////////////////////////////////////////////
using autoware::common::types::PointXYZIF;
using autoware::common::types::float32_t;
using autoware::perception::filters::ray_ground_classifier::PointPtrBlock;

using std::placeholders::_1;

using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;
using autoware::common::lidar_utils::init_pcl_msg;
using autoware::common::lidar_utils::add_point_to_cloud_raw;

RayGroundClassifierCloudNode::RayGroundClassifierCloudNode(
  const rclcpp::NodeOptions & node_options)
: Node("ray_ground_classifier", node_options),
#ifdef _OPENMP
  m_classifiers(omp_get_max_threads(), ray_ground_classifier::RayGroundClassifier(
      ray_ground_classifier::Config {
          static_cast<float32_t>(declare_parameter("classifier.sensor_height_m").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_local_slope_deg").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_global_slope_deg").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.nonground_retro_thresh_deg").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.min_height_thresh_m").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_global_height_thresh_m").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_last_local_ground_thresh_m").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_provisional_ground_distance_m").get<float32_t>())
        })),
#else
  m_classifier(ray_ground_classifier::Config{
          static_cast<float32_t>(declare_parameter("classifier.sensor_height_m").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_local_slope_deg").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_global_slope_deg").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.nonground_retro_thresh_deg").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.min_height_thresh_m").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_global_height_thresh_m").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_last_local_ground_thresh_m").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "classifier.max_provisional_ground_distance_m").get<float32_t>())
        }),
#endif
  m_aggregator(ray_ground_classifier::RayAggregator::Config{
          static_cast<float32_t>(declare_parameter(
            "aggregator.min_ray_angle_rad").get<float32_t>()),
          static_cast<float32_t>(declare_parameter(
            "aggregator.max_ray_angle_rad").get<float32_t>()),
          static_cast<float32_t>(declare_parameter("aggregator.ray_width_rad").get<float32_t>()),
          static_cast<std::size_t>(
            declare_parameter("aggregator.max_ray_points").get<std::size_t>())
        }),
  m_pcl_size(static_cast<std::size_t>(declare_parameter("pcl_size").get<std::size_t>())),
  m_frame_id(declare_parameter("frame_id").get<std::string>().c_str()),
  m_has_failed(false),
  m_timeout(std::chrono::milliseconds{declare_parameter("cloud_timeout_ms").get<uint16_t>()}),
  m_raw_sub_ptr(create_subscription<PointCloud2>(
      "points_in",
      rclcpp::QoS(10), std::bind(&RayGroundClassifierCloudNode::callback, this, _1))),
  m_ground_pub_ptr(create_publisher<PointCloud2>(
      "points_ground", rclcpp::QoS(10))),
  m_nonground_pub_ptr(create_publisher<PointCloud2>(
      "points_nonground", rclcpp::QoS(10))),
  m_ground_pc_idx{0},
  m_nonground_pc_idx{0}
{
  // initialize messages
  init_pcl_msg(m_ground_msg, m_frame_id.c_str(), m_pcl_size);
  init_pcl_msg(m_nonground_msg, m_frame_id.c_str(), m_pcl_size);
}
////////////////////////////////////////////////////////////////////////////////
void
RayGroundClassifierCloudNode::callback(const PointCloud2::SharedPtr msg)
{
  PointXYZIF pt_tmp;
  pt_tmp.id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
  const ray_ground_classifier::PointXYZIFR eos_pt{&pt_tmp};

  try {
    // Reset messages and aggregator to ensure they are in a good state
    reset();
    // Verify header
    if (msg->header.frame_id != m_ground_msg.header.frame_id) {
      throw std::runtime_error(
              "RayGroundClassifierCloudNode: raw topic from unexpected "
              "frame (expected '" + m_ground_msg.header.frame_id +
              "', got '" + msg->header.frame_id + "')");
    }
    // Verify the consistency of PointCloud msg
    const auto data_length = msg->width * msg->height * msg->point_step;
    if ((msg->data.size() != msg->row_step) || (data_length != msg->row_step)) {
      throw std::runtime_error("RayGroundClassifierCloudNode: Malformed PointCloud2");
    }
    // Verify the point cloud format and assign correct point_step
    if (!has_intensity_and_throw_if_no_xyz(msg)) {
      RCLCPP_WARN(this->get_logger(),
        "RayGroundClassifierNode Warning: PointCloud doesn't have intensity field");
    }
    // Harvest timestamp
    m_nonground_msg.header.stamp = msg->header.stamp;
    m_ground_msg.header.stamp = msg->header.stamp;
    // Add all points to aggregator
    // Iterate through the data, but skip intensity in case the point cloud does not have it.
    // For example:
    //
    // point_step = 4
    // x y z i a b c x y z i a b c
    // ^------       ^------
    size_t num_ready = 0;
    bool8_t has_encountered_unknown_exception = false;
    bool8_t abort = false;
    #pragma omp parallel shared(m_aggregator, num_ready, has_encountered_unknown_exception, abort)
    {
      std::size_t point_step = msg->point_step;
      #pragma omp for schedule(dynamic, 50)
      for (std::size_t idx = 0U; idx < msg->data.size(); idx += point_step) {
        #pragma omp flush (abort)
        if (abort) {
          continue;
        }
        try {
          PointXYZIF * pt;
          // TODO(c.ho) Fix below deviation after #2131 is in
          //lint -e{925, 9110} Need to convert pointers and use bit for external API NOLINT
          pt = reinterpret_cast<PointXYZIF *>(&msg->data[idx]);
          // don't bother inserting the points almost (0,0).
          // Too many of those makes the bin 0 overflow
          if ((fabsf(pt->x) > autoware::common::types::FEPS) ||
            (fabsf(pt->y) > autoware::common::types::FEPS))
          {
            if (!m_aggregator.insert(pt)) {
              #ifdef _OPENMP
              RCLCPP_WARN_ONCE(this->get_logger(),
                "early end of scan are ignored in parallel mode");
              #else
              m_aggregator.end_of_scan();
              #endif
            }
          } else {
            uint32_t local_nonground_pc_idx;
            #pragma omp atomic capture
            local_nonground_pc_idx = m_nonground_pc_idx++;
            if (!add_point_to_cloud_raw(m_nonground_msg, *pt, local_nonground_pc_idx)) {
              throw std::runtime_error(
                      "RayGroundClassifierNode: Overran nonground msg point capacity");
            }
          }
        } catch (const std::runtime_error & e) {
          m_has_failed = true;
          #pragma omp critical (get_logger)
          RCLCPP_INFO(this->get_logger(), e.what());
          abort = true;
        } catch (const std::exception & e) {
          m_has_failed = true;
          #pragma omp critical (get_logger)
          RCLCPP_INFO(this->get_logger(), e.what());
          abort = true;
        } catch (...) {
          #pragma omp critical (get_logger)
          RCLCPP_INFO(this->get_logger(),
            "RayGroundClassifierCloudNode has encountered an unknown failure");
          abort = true;
          has_encountered_unknown_exception = true;
        }
      }
      // Implicit omp barrier here

      // if abort, we skip all remaining the parallel work to be able to return/throw
      #pragma omp flush (abort)
      if (!abort) {
        #pragma omp single
        {
          m_aggregator.end_of_scan();
          num_ready = m_aggregator.get_ready_ray_count();
        }
        // Implicit omp barrier and flush(num_ready) here

        // Partition each ray
        #pragma omp for
        for (size_t i = 0; i < num_ready; i++) {
          #pragma omp flush (abort)
          if (abort) {
            continue;
          }
          // Note: if an exception occurs in this loop, the aggregator can get into a bad state
          // (e.g. overrun capacity)
          PointPtrBlock ground_blk;
          PointPtrBlock nonground_blk;
          try {
            auto ray = m_aggregator.get_next_ray();
            #ifdef _OPENMP
            m_classifiers[omp_get_thread_num()].partition(ray, ground_blk, nonground_blk);
            #else
            // partition: should never fail, guaranteed to have capacity via other checks
            m_classifier.partition(ray, ground_blk, nonground_blk);
            #endif
            // Add ray to point clouds
            for (auto & ground_point : ground_blk) {
              uint32_t local_ground_pc_idx;
              #pragma omp atomic capture
              local_ground_pc_idx = m_ground_pc_idx++;
              if (!add_point_to_cloud_raw(m_ground_msg, *ground_point,
                local_ground_pc_idx))
              {
                throw std::runtime_error(
                        "RayGroundClassifierNode: Overran ground msg point capacity");
              }
            }
            for (auto & nonground_point : nonground_blk) {
              uint32_t local_nonground_pc_idx;
              #pragma omp atomic capture
              local_nonground_pc_idx = m_nonground_pc_idx++;
              if (!add_point_to_cloud_raw(m_nonground_msg, *nonground_point,
                local_nonground_pc_idx))
              {
                throw std::runtime_error(
                        "RayGroundClassifierNode: Overran nonground msg point capacity");
              }
            }
          } catch (const std::runtime_error & e) {
            m_has_failed = true;
            #pragma omp critical (get_logger)
            RCLCPP_INFO(this->get_logger(), e.what());
            abort = true;
          } catch (const std::exception & e) {
            m_has_failed = true;
            #pragma omp critical (get_logger)
            RCLCPP_INFO(this->get_logger(), e.what());
            abort = true;
          } catch (...) {
            #pragma omp critical (get_logger)
            RCLCPP_INFO(this->get_logger(),
              "RayGroundClassifierCloudNode has encountered an unknown failure");
            abort = true;
            has_encountered_unknown_exception = true;
          }
        }
        // Implicit omp barrier here
      }
    }

    if (has_encountered_unknown_exception) {
      m_has_failed = true;
      RCLCPP_INFO(this->get_logger(),
        "RayGroundClassifierCloudNode has encountered an unknown failure");
      throw;
    }

    if (abort) {
      return;
    }


    // Resize the clouds down to their actual sizes.
    autoware::common::lidar_utils::resize_pcl_msg(m_ground_msg, m_ground_pc_idx);
    autoware::common::lidar_utils::resize_pcl_msg(m_nonground_msg, m_nonground_pc_idx);
    // publish: nonground first for the possible microseconds of latency
    m_nonground_pub_ptr->publish(m_nonground_msg);
    m_ground_pub_ptr->publish(m_ground_msg);
  } catch (const std::runtime_error & e) {
    m_has_failed = true;
    RCLCPP_INFO(this->get_logger(), e.what());
  } catch (const std::exception & e) {
    m_has_failed = true;
    RCLCPP_INFO(this->get_logger(), e.what());
  } catch (...) {
    RCLCPP_INFO(this->get_logger(),
      "RayGroundClassifierCloudNode has encountered an unknown failure");
    throw;
  }
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifierCloudNode::reset()
{
  // reset aggregator: Needed in case an error is thrown during partitioning of cloud
  //                   which would lead to filled rays and overflow during next callback
  m_aggregator.reset();
  // reset messages
  autoware::common::lidar_utils::reset_pcl_msg(m_ground_msg, m_pcl_size, m_ground_pc_idx);
  autoware::common::lidar_utils::reset_pcl_msg(m_nonground_msg, m_pcl_size, m_nonground_pc_idx);
}
}  // namespace ray_ground_classifier_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::ray_ground_classifier_nodes::RayGroundClassifierCloudNode)
