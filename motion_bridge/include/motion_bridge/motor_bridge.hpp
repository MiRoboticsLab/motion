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
#ifndef MOTION_BRIDGE__MOTOR_BRIDGE_HPP_
#define MOTION_BRIDGE__MOTOR_BRIDGE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <string>
#include <memory>
#include "motion_action/motion_macros.hpp"
#include "protocol/lcm/danger_states_lcmt.hpp"
#include "protocol/srv/motor_temp.hpp"
namespace cyberdog
{
namespace motion
{
class MotorBridge
{
public:
  explicit MotorBridge(const rclcpp::Node::SharedPtr node);
  ~MotorBridge() {}
  void Spin();

private:
  void ReadLcm(
    const lcm::ReceiveBuffer *, const std::string &,
    const danger_states_lcmt * msg);
  void HandleMotorTempCallback(
    const protocol::srv::MotorTemp_Request::SharedPtr req,
    protocol::srv::MotorTemp_Response::SharedPtr res);
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<protocol::srv::MotorTemp>::SharedPtr motor_temp_srv_ {nullptr};
  std::shared_ptr<lcm::LCM> lcm_subscribe_instance_;
  danger_states_lcmt motor_temp_;
  std::thread lcm_handle_thread_;
};  // class MotorBridge
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_BRIDGE__MOTOR_BRIDGE_HPP_
