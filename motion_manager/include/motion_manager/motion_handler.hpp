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
#ifndef MOTION_MANAGER__MOTION_HANDLER_HPP_
#define MOTION_MANAGER__MOTION_HANDLER_HPP_
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "motion_action/motion_action.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/lcm/robot_control_cmd_lcmt.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"

namespace cyberdog
{
namespace motion
{

/**
 * @brief 接收运控板返回数据，并进行解析、管理和分发
 *
 */
class MotionHandler final
{
  using LcmResponse = robot_control_response_lcmt;
  using MotionServoResponseMsg = protocol::msg::MotionServoResponse;
  using MotionStatusMsg = protocol::msg::MotionStatus;

public:
  explicit MotionHandler(rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr publisher);
  ~MotionHandler();

public:
  /* recv api */
  // void ServoResponse();
  void RegisterUpdate(std::function<void(MotionStatusMsg::SharedPtr)> f)
  {
    motion_response_func = f;
  }
  void Update();
  void Checkout(MotionStatusMsg::SharedPtr motion_status_ptr);
  bool CheckMotionID(int32_t motion_id);

private:
  /* ros members */
  std::shared_ptr<LcmResponse> lcm_response_ {nullptr};
  std::thread servo_response_thread_;
  std::function<void(MotionStatusMsg::SharedPtr)> motion_response_func;
};  // class MotionHandler
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_MANAGER__MOTION_HANDLER_HPP_
