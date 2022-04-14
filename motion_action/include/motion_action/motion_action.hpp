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
#ifndef MOTION_ACTION__MOTION_ACTION_HPP_
#define MOTION_ACTION__MOTION_ACTION_HPP_
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <iostream>
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"

namespace cyberdog
{
namespace motion
{
using MotionServoCmdMsg = protocol::msg::MotionServoCmd;
using LcmResponse = robot_control_response_lcmt;

class MotionAction final
{
public:
  MotionAction();
  ~MotionAction();

public:
  void Execute(const MotionServoCmdMsg::SharedPtr msg);
  void RegisterFeedback(std::function<void(LcmResponse *)> feedback);
  bool Init();
  bool SelfCheck();

private:
  std::thread control_thread_;
  std::function<void(LcmResponse *)> feedback_func_;
};  // class MotionAction
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_ACTION__MOTION_ACTION_HPP_
