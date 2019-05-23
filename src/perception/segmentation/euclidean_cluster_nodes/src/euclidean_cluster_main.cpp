// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

#include <iostream>
//lint -e537 NOLINT repeated include file... cpplint disagrees with pclint
#include <memory>
#include "euclidean_cluster_nodes/euclidean_cluster_node.hpp"

int32_t main(const int32_t argc, char ** const argv)
{
  int32_t ret = 0;
  try {
    rclcpp::init(argc, argv);
    using autoware::perception::segmentation::euclidean_cluster_nodes::EuclideanClusterNode;
    const auto nd_ptr = std::make_shared<EuclideanClusterNode>("euclidean_cluster_node");

    rclcpp::spin(nd_ptr);

    if (!rclcpp::shutdown()) {
      throw std::runtime_error{"Could not shut down rclcpp"};
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    ret = 2;
  } catch (...) {
    std::cerr << "Unexpected exception caught! Exiting\n";
    ret = 255;
  }

  return ret;
}
