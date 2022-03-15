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
#include "motion_manager/motion_decision.hpp"

cyberdog::motion::MotionDecision::MotionDecision(std::shared_ptr<MotionAction> action_ptr, std::shared_ptr<MotionHandler> handler_ptr)
  : action_ptr_ (action_ptr), handler_ptr_ (handler_ptr)
{
  action_ptr = std::make_shared<cyberdog::motion::MotionAction>();
}
cyberdog::motion::MotionDecision::~MotionDecision() {}

void cyberdog::motion::MotionDecision::Config() {}
bool cyberdog::motion::MotionDecision::Init() {}
// bool cyberdog::motion::MotionDecision::CheckModeValid() {
//   return true;
// }
void cyberdog::motion::MotionDecision::Execute(const MotionCmdMsg::SharedPtr msg) {
  if(! IsStateValid()) {
    return;
  }
  if(! IsModeValid()) {
    return;
  }

  handler_ptr_->Update();
  action_ptr_->Execute();
}