// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

#include <memory>
#include <string>

#include "pure_pursuit_nodes/pure_pursuit_node.hpp"

int32_t main(const int32_t argc, char * argv[])
{
  int32_t ret = 0;
  try {
    rclcpp::init(argc, argv);
    using autoware::motion::control::pure_pursuit_nodes::PurePursuitNode;
    const auto nd_ptr = std::make_shared<PurePursuitNode>("pure_pursuit_node");

    rclcpp::spin(nd_ptr);

    if (!rclcpp::shutdown()) {
      throw std::runtime_error{"Could not shutdown rclcpp"};
    }
  } catch (const std::exception & e) {
    std::cerr << e.what();
    ret = __LINE__;
  } catch (...) {
    std::cerr << "Unknown error occured";
    ret = __LINE__;
  }

  return ret;
}
