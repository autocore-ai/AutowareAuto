// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief main function for lanelet2_map_provider

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <memory>
#include <chrono>

#include "autoware_auto_msgs/srv/had_map_service.hpp"
#include "autoware_auto_msgs/msg/had_map_bin.hpp"
#include "had_map_utils/had_map_conversion.hpp"
#include "lanelet2_map_provider/lanelet2_map_provider.hpp"

using namespace std::chrono_literals;

int32_t main(const int32_t argc, char ** const argv)
{
  int32_t ret;
  try {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("HAD_Map_Clinet");
    rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr client =
      node->create_client<autoware_auto_msgs::srv::HADMapService>("HAD_Map_Service");

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        std::cerr << "Interrupted while waiting for the service. Exiting.\n";
        return 0;
      }
    }

    auto request = std::make_shared<autoware_auto_msgs::srv::HADMapService_Request>();
    request->requested_primitives.push_back(
      autoware_auto_msgs::srv::HADMapService_Request::FULL_MAP);
    request->requested_primitives.push_back(
      autoware_auto_msgs::srv::HADMapService_Request::DRIVEABLE_GEOMETRY);

    request->geom_upper_bound.push_back(1.0);
    request->geom_upper_bound.push_back(1.0);
    request->geom_upper_bound.push_back(0.0);
    request->geom_lower_bound.push_back(-1.0);
    request->geom_lower_bound.push_back(-1.0);
    request->geom_lower_bound.push_back(0.0);

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      std::cerr << "Success\n";
    } else {
      std::cerr << "Failure\n";
    }
    rclcpp::shutdown();

    std::shared_ptr<lanelet::LaneletMap> sub_map = std::make_shared<lanelet::LaneletMap>();

    autoware_auto_msgs::msg::HADMapBin msg = result.get()->map;
    autoware::common::had_map_utils::fromBinaryMsg(msg, sub_map);
    std::cerr << "had map client - size of recieved map " << sub_map->size() << "\n";
    ret = 0;
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    ret = 1;
  } catch (...) {
    std::cerr << "Unknown error occured" << "\n";
    ret = 255;
  }
  return ret;
}
