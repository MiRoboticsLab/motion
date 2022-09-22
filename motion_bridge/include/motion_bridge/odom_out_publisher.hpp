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
#ifndef MOTION_BRIDGE__ODOM_OUT_PUBLISHER_HPP_
#define MOTION_BRIDGE__ODOM_OUT_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include <protocol/lcm/heightmap_t.hpp>
#include <protocol/lcm/localization_lcmt.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <nav_msgs/msg/odometry.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <condition_variable>
#include <string>
#include <memory>
#include "motion_action/motion_macros.hpp"

namespace cyberdog
{
namespace motion
{

class OdomOutPublisher
{
public:
  explicit OdomOutPublisher(const rclcpp::Node::SharedPtr node);
  ~OdomOutPublisher() {}
  void Spin();

private:
  void OdomLCMCabllback(
    const lcm::ReceiveBuffer *, const std::string &,
    const localization_lcmt * msg);
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<lcm::LCM> lcm_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr leg_odom_publisher_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::TransformStamped transform_stamped_;
  std::string map_frame_{"map"}, odom_frame_{"odom"}, base_frame_{"base_link_leg"};
  std::condition_variable cv_;
  std::mutex mutex_;
  bool ready_publish_;
  bool tf_pub_;
};  // class OdomOutPublisher
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_BRIDGE__ODOM_OUT_PUBLISHER_HPP_
