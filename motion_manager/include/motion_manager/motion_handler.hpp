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
#include "motion_action/motion_action.hpp"
#include "protocol/msg/motion_cmd.hpp"
#include "protocol/lcm/robot_control_cmd_lcmt.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"

namespace cyberdog
{
namespace motion
{
using MotionCmdMsg = protocol::msg::MotionCmd;
using LcmResponse = robot_control_response_lcmt;
/**
 * @brief 接收运控板返回数据，并进行解析、管理和分发
 * 
 */
class MotionHandler final
{
public:
  MotionHandler();
  ~MotionHandler();

public: /* recv api */
  
  void Update();
  void Checkout(LcmResponse * response);

private: /* ros members */
  std::shared_ptr<LcmResponse> lcm_response_ {nullptr};
};  // class MotionHandler
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_MANAGER__MOTION_HANDLER_HPP_