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
#ifndef MOTION_BRIDGE__ELEVATION_BRIDGE_HPP_
#define MOTION_BRIDGE__ELEVATION_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <lcm/lcm-cpp.hpp>
#include <protocol/lcm/heightmap_t.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <cyberdog_common/cyberdog_log.hpp>
#include <string>
#include <memory>
#include "motion_action/motion_macros.hpp"

namespace cyberdog
{
namespace motion
{
constexpr int8_t SIZE_X = 80;
constexpr int8_t SIZE_Y = 80;
constexpr float RESOLUTION = 0.05;
constexpr const char * LAYER_ELEVATION = "elevation";
constexpr const char * LAYER_TRAVERSABILITY = " ";
class ElevationBridge
{
public:
  explicit ElevationBridge(const rclcpp::Node::SharedPtr node);
  ~ElevationBridge() {}
  void Spin();

private:
  void GridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  bool GetTransform(
    Eigen::Isometry3d & transform, const std::string & target_frame,
    const std::string & source_frame);
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<lcm::LCM> lcm_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_sub_;
  heightmap_t elevation_;
  std::string map_frame_, odom_frame_, base_frame_, cam_frame_;
};  // class ElevationBridge
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_BRIDGE__ELEVATION_BRIDGE_HPP_
