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
#ifndef MOTION_MANAGER__MOTION_MANAGER_HPP_
#define MOTION_MANAGER__MOTION_MANAGER_HPP_
#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <memory>
#include "pluginlib/class_loader.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/lcm/robot_control_cmd_lcmt.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"
#include "manager_base/manager_base.hpp"
#include "motion_manager/motion_decision.hpp"
#include "motion_action/motion_action.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace motion
{
using MotionServoCmdMsg = protocol::msg::MotionServoCmd;
using MotionServoResponseMsg = protocol::msg::MotionServoResponse;
using MotionResultSrv = protocol::srv::MotionResultCmd;

class MotionManager final : public manager::ManagerBase
{
public:
  explicit MotionManager(const std::string & name);
  ~MotionManager();

  void Config() override;
  bool Init() override;
  void Run() override;
  bool SelfCheck() override;

public:
  void OnError() override;
  void OnLowPower() override;
  void OnSuspend() override;
  void OnProtected() override;
  void OnActive() override;

private:
  bool IsStateValid();
  void MotionServoCmdCallback(const MotionServoCmdMsg::SharedPtr msg);
  void MotionResultCmdCallback(
    const MotionResultSrv::Request::SharedPtr request,
    MotionResultSrv::Response::SharedPtr response);

private:
  std::string name_;
  std::shared_ptr<MotionDecision> decision_ptr_ {nullptr};
  // std::shared_ptr<MotionHandler> handler_ptr_ {nullptr};
  // std::shared_ptr<MotionAction> action_ptr_ {nullptr};

private:
  rclcpp::Subscription<MotionServoCmdMsg>::SharedPtr motion_servo_sub_ {nullptr};
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr motion_servo_pub_ {nullptr};
  rclcpp::Service<MotionResultSrv>::SharedPtr motion_result_srv_ {nullptr};
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
};  // class MotionManager
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_MANAGER_HPP_
