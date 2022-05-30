// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <memory>
#include "motion_bridge/elevation_bridge.hpp"

namespace cyberdog
{
namespace motion_bridge
{
ElevationBridge::ElevationBridge(const rclcpp::Node::SharedPtr node)
: map_frame_("map"),
  odom_frame_("odom"),
  base_frame_("base_link")
{
  node_ = node;
  gridmap_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
    "elevation_map_raw",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ElevationBridge::GridMapCallback, this, std::placeholders::_1));
  lcm_ = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7667?ttl=255");
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    node_->get_node_base_interface(),
    node_->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_, std::make_shared<rclcpp::Node>(std::string(node_->get_name()) + "_tf_listener"));
  std::thread{[this]() {
      while (lcm_->good()) {
        lcm_->publish("local_heightmap", &elevation_);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
  }.detach();
}

void ElevationBridge::Spin()
{
  rclcpp::spin(node_);
  rclcpp::shutdown();
}

bool ElevationBridge::GetTransform(
  Eigen::Isometry3d & transform, const std::string & target_frame,
  const std::string & source_frame)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped =
      tf2_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & e) {
    WARN("%s", e.what());
    return false;
  }
  transform.translation()(0) = transform_stamped.transform.translation.x;
  transform.translation()(1) = transform_stamped.transform.translation.y;
  transform.translation()(2) = transform_stamped.transform.translation.z;
  auto q = transform_stamped.transform.rotation;
  Eigen::Quaterniond quat(q.x, q.y, q.z, q.w);
  transform.rotate(quat);
  return true;
}

void ElevationBridge::GridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  double resolution = msg->info.resolution;
  if (std::abs(resolution - RESOLUTION) > 0.001) {
    ERROR("Resolution error");
    // return;
  }
  double length_x = resolution * SIZE_X;
  double length_y = resolution * SIZE_Y;
  if (msg->info.length_x < length_x || msg->info.length_y < length_y) {
    ERROR("Lenth error");
    // return;
  }
  grid_map::GridMap map, submap;
  if (!grid_map::GridMapRosConverter::fromMessage(*msg, map)) {
    ERROR("Cannot convert grid map from ros msg");
    // return;
  }
  if (msg->header.frame_id != odom_frame_) {
    // trans msg to odom frame
    Eigen::Isometry3d transform;
    if (!GetTransform(transform, odom_frame_, msg->header.frame_id)) {
      ERROR("Cannot transform");
    }
    map = map.getTransformedMap(transform, LAYER_ELEVATION, odom_frame_);
  }
  double center_pos_x = map.getPosition()(0);
  double center_pos_y = map.getPosition()(1);
  elevation_.robot_loc[0] = center_pos_x;
  elevation_.robot_loc[1] = center_pos_y;

  /**************************************************************************/
  // bool success = false;
  // submap = map.getSubmap(map.getPosition(), grid_map::Length(length_x, length_y), success);
  // if (!success) {
  //   ERROR("Cannot get submap");
  // }
  // INFO("map size: %d, %d", map.getSize()(0), map.getSize()(1));
  // INFO("submap size: %d, %d", submap.getSize()(0), submap.getSize()(1));
  // if (submap.getSize()(0) != SIZE_X || submap.getSize()(1) != SIZE_Y) {
  //   ERROR("Submap size error");
  // }
  // for (grid_map::GridMapIterator iter(submap); iter.isPastEnd(); ++iter) {
  //   for (int8_t i = 0; i < SIZE_X; i++) {
  //     for (int8_t j = 0; j < SIZE_Y; j++) {
  //       elevation_.map[i][j] = map.at(LAYER_ELEVATION, *iter);
  //       ++iter;
  //     }
  //   }
  // }
  /**************************************************************************/
  double submap_left_top_x = center_pos_x + length_x / 2;
  double submap_left_top_y = center_pos_y + length_y / 2;
  grid_map::Index submap_left_top_index;
  if (!map.getIndex(
      grid_map::Position(submap_left_top_x, submap_left_top_y),
      submap_left_top_index))
  {
    ERROR("Cannot get lefttop corner");
    // return;
  }
  grid_map::SubmapIterator iter(map, submap_left_top_index, grid_map::Size(SIZE_X, SIZE_Y));
  for (int8_t i = 0; i < SIZE_X; i++) {
    for (int8_t j = 0; j < SIZE_Y; j++) {
      elevation_.map[i][j] = map.at(LAYER_ELEVATION, *iter);
      ++iter;
    }
  }
  /**************************************************************************/
}
}
}
