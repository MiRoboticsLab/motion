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
#ifndef MOTION_MANAGER__MOTION_DECISION_HPP_
#define MOTION_MANAGER__MOTION_DECISION_HPP_
#include <thread>
#include <string>
#include <memory>
#include "motion_action/motion_action.hpp"
#include "motion_manager/motion_handler.hpp"
#include "protocol/msg/motion_cmd.hpp"

namespace cyberdog
{
namespace motion
{
class MotionDecision final
{
public:
  MotionDecision(
    std::shared_ptr<MotionAction> action_ptr,
    std::shared_ptr<cyberdog::motion::MotionHandler> handler_ptr);
  ~MotionDecision();

  void Config();
  bool Init();
  void Execute(const MotionCmdMsg::SharedPtr msg);
  inline void SetMode(uint8_t mode)
  {
    motion_control_mode_ = mode;
  }

private:
  inline bool IsStateValid()
  {
    return true;
  }

  inline bool IsModeValid()
  {
    return true;
  }

private:
  std::shared_ptr<MotionAction> action_ptr_ {nullptr};
  std::shared_ptr<MotionHandler> handler_ptr_ {nullptr};
  uint8_t motion_control_mode_ {0};
};  // class MotionDecision
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_DECISION_HPP_
