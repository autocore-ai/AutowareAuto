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

#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>

#include <chrono>
#include <utility>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lanelet2_map_provider/lanelet2_map_provider.hpp"
#include "lanelet2_map_provider/lanelet2_map_provider_node.hpp"
#include "gtest/gtest.h"

using namespace std::chrono_literals;

lanelet::LaneletMap getALaneletMap()
{
  lanelet::LineString3d ls1(lanelet::utils::getId(),
    {lanelet::Point3d{lanelet::utils::getId(), 0, 0, 0},
      lanelet::Point3d{lanelet::utils::getId(), 0, 1, 0},
      lanelet::Point3d{lanelet::utils::getId(), 0, 2, 0}});
  lanelet::LineString3d ls2(lanelet::utils::getId(),
    {lanelet::Point3d{lanelet::utils::getId(), 1, 0, 0},
      lanelet::Point3d{lanelet::utils::getId(), 1, 1, 0},
      lanelet::Point3d{lanelet::utils::getId(), 1, 2, 0}});
  lanelet::Lanelet ll(lanelet::utils::getId(), ls1, ls2);
  return std::move(*lanelet::utils::createMap({ll}));
}

TEST(TestLanelet2MapProvider, BasicTest) {
  std::cerr << "basic test\n";

  // build a simple lanelet map
  lanelet::LaneletMap lanelet_map = getALaneletMap();
  lanelet::PointLayer & points = lanelet_map.pointLayer;
  size_t n_pts = points.size();
  std::cerr << "number of points " << n_pts << "\n";
  //  EXPECT_EQ(autoware::lanelet2_map_provider::print_hello(), 0);

  // save it to tempory storage
  std::string lanelet2_map_file = "lanelet2_test.osm";
  write(lanelet2_map_file, lanelet_map);
  autoware::lanelet2_map_provider::Lanelet2MapProvider map_provider(lanelet2_map_file, {37.0,
      -120.0, 16.0});
  remove(lanelet2_map_file.c_str());
}

TEST(TestLanelet2MapProviderNode, TestService) {
  std::cerr << "test node\n";
  std::string program_name = "test_node";
  char * argv[] = {strdup("test_node"), NULL};
  int argc = sizeof(argv) / sizeof(char *) - 1;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // build a simple lanelet map
  lanelet::LaneletMap lanelet_map = getALaneletMap();
  lanelet::PointLayer & points = lanelet_map.pointLayer;
  size_t n_pts = points.size();
  std::cerr << "number of points " << n_pts << "\n";
  //  EXPECT_EQ(autoware::lanelet2_map_provider::print_hello(), 0);

  // save it to tempory storage
  std::string lanelet2_map_file = "/tmp/lanelet2_test.osm";
  write(lanelet2_map_file, lanelet_map);

  /*
  std::cerr << "after write map to file\n";
  const auto map_node_ptr =
    std::make_shared<autoware::lanelet2_map_provider::Lanelet2MapProviderNode>(
    options, lanelet2_map_file);
  std::cerr << " after run node\n";
  //  autoware::lanelet2_map_provider::Lanelet2MapProviderNode map_node(options, lanelet2_map_file);
  rclcpp::spin_some(map_node_ptr);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("HAD_Map_Clinet");
  rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr client =
    node->create_client<autoware_auto_msgs::srv::HADMapService>("HAD_Map_Service");

  auto request = std::make_shared<autoware_auto_msgs::srv::HADMapService_Request>();
  request->requested_primitives.push_back(1);
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      std::cerr << "Interrupted while waiting for the service. Exiting.\n";
      return;
    }
    std::cerr << "service not available, waiting again...\n";
    rclcpp::spin_some(map_node_ptr);
  }

  auto result = client->async_send_request(request);
  rclcpp::spin_some(map_node_ptr);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    std::cerr << "Success\n";
  } else {
    std::cerr << "Failure\n";
  }
  */
  rclcpp::shutdown();
}
