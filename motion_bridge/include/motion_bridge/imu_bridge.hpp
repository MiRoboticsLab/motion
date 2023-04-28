// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef MOTION_BRIDGE__IMU_BRIDGE_HPP_
#define MOTION_BRIDGE__IMU_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <cyberdog_common/cyberdog_log.hpp>
#include <string>
#include <memory>
#include "motion_action/motion_macros.hpp"
#include "protocol/lcm/microstrain_lcmt.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"
#include "sensor_msgs/msg/imu.hpp"
namespace cyberdog
{
namespace motion
{
constexpr int8_t SIZE_X = 80;
constexpr int8_t SIZE_Y = 80;
constexpr float RESOLUTION = 0.05;
constexpr const char * LAYER_ELEVATION = "elevation";
constexpr const char * LAYER_TRAVERSABILITY = " ";
class ImuBridge
{
public:
  explicit ImuBridge(const rclcpp::Node::SharedPtr node);
  ~ImuBridge() {}
  void Spin();

private:
  void ReadLcm(
    const lcm::ReceiveBuffer *, const std::string &,
    const microstrain_lcmt * msg);
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<lcm::LCM> lcm_subscribe_instance_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_ {nullptr};
  sensor_msgs::msg::Imu::SharedPtr imu_ros_data_ {nullptr};
  microstrain_lcmt imu_lcm_data_;
  robot_control_response_lcmt response_lcm_data_;
  std::thread topic_publish_thread_, lcm_handle_thread_;
  std::string imu_frame_{"imu"};
};  // class ImuBridge
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_BRIDGE__IMU_BRIDGE_HPP_
