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
#include "motion_action/motion_action.hpp"

cyberdog::motion::MotionAction::MotionAction() {}

cyberdog::motion::MotionAction::~MotionAction() {}

void cyberdog::motion::MotionAction::Execute(const MotionServoCmdMsg::SharedPtr msg)
{
  // Checkout mode global, send msg continuously
  (void) msg;
}

bool cyberdog::motion::MotionAction::Init()
{
  return true;
}

bool cyberdog::motion::MotionAction::SelfCheck()
{
  return true;
}

void cyberdog::motion::MotionAction::RegisterFeedback(std::function<void(LcmResponse *)> feedback)
{
  feedback_func_ = feedback;
}
