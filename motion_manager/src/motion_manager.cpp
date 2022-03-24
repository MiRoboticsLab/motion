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
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_manager/motion_manager.hpp"

cyberdog::motion::MotionManager::MotionManager(const std::string & name)
: manager::ManagerBase(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
  action_ptr_ = std::make_shared<MotionAction>();
  handler_ptr_ = std::make_shared<MotionHandler>();
  decision_ptr_ = std::make_shared<MotionDecision>(action_ptr_, handler_ptr_);
}

cyberdog::motion::MotionManager::~MotionManager()
{}

void cyberdog::motion::MotionManager::Config()
{
  INFO("get info from configure");
}

bool cyberdog::motion::MotionManager::Init()
{
  // register manager base functions
  action_ptr_->RegisterFeedback(
    std::bind(
      &MotionHandler::Checkout, this->handler_ptr_,
      std::placeholders::_1));
  motion_cmd_sub_ = node_ptr_->create_subscription<MotionCmdMsg>(
    "motion_cmd", rclcpp::SystemDefaultsQoS(),
    std::bind(&MotionManager::MotionCmdSubCallback, this, std::placeholders::_1));
  return true;
}

void cyberdog::motion::MotionManager::Run()
{
  rclcpp::spin(node_ptr_);
  rclcpp::shutdown();
}

bool cyberdog::motion::MotionManager::SelfCheck()
{
  // check all motions from config
  return true;
}

bool cyberdog::motion::MotionManager::IsStateValid()
{
  // check state from behavior tree
  return true;
}

void cyberdog::motion::MotionManager::OnError()
{
  INFO("on error");
}

void cyberdog::motion::MotionManager::OnLowPower()
{
  INFO("on lowpower");
}

void cyberdog::motion::MotionManager::OnSuspend()
{
  INFO("on suspend");
}

void cyberdog::motion::MotionManager::OnProtected()
{
  INFO("on protect");
}

void cyberdog::motion::MotionManager::OnActive()
{
  INFO("on active");
}

void cyberdog::motion::MotionManager::MotionCmdSubCallback(const MotionCmdMsg::SharedPtr msg)
{
  if (!IsStateValid()) {
    INFO("motion state invalid with current state");
    return;
  }

  decision_ptr_->Execute(msg);
}
