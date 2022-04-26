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
}

cyberdog::motion::MotionManager::~MotionManager()
{}

void cyberdog::motion::MotionManager::Config()
{
  INFO("Get info from configure");
  action_ptr_ = std::make_shared<MotionAction>();
  handler_ptr_ = std::make_shared<MotionHandler>(motion_servo_pub_);
  decision_ptr_ = std::make_shared<MotionDecision>(action_ptr_, handler_ptr_);
}

bool cyberdog::motion::MotionManager::Init()
{
  INFO("Init on call");
  if (node_ptr_ == nullptr) {
    ERROR("Init failed with nullptr at ros node!");
    return false;
  }

  decision_ptr_->Init(motion_servo_pub_);
  action_ptr_->Init();

  action_ptr_->RegisterFeedback(
    std::bind(
      &MotionHandler::Checkout, this->handler_ptr_,
      std::placeholders::_1));
  motion_servo_sub_ = node_ptr_->create_subscription<MotionServoCmdMsg>(
    "motion_servo_cmd", rclcpp::SystemDefaultsQoS(),
    std::bind(&MotionManager::MotionServoCmdCallback, this, std::placeholders::_1));
  motion_servo_pub_ = node_ptr_->create_publisher<MotionServoResponseMsg>(
    "motion_servo_response",
    10);
  motion_result_srv_ =
    node_ptr_->create_service<MotionResultSrv>(
    "motion_result_cmd",
    std::bind(
      &MotionManager::MotionResultCmdCallback, this, std::placeholders::_1,
      std::placeholders::_2));


  return true;
}

void cyberdog::motion::MotionManager::Run()
{
  INFO("Running on...");
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

void cyberdog::motion::MotionManager::MotionServoCmdCallback(const MotionServoCmdMsg::SharedPtr msg)
{
  if (!IsStateValid()) {
    INFO("motion state invalid with current state");
    return;
  }

  decision_ptr_->Servo(msg);
}

void cyberdog::motion::MotionManager::MotionResultCmdCallback(
  const MotionResultSrv::Request::SharedPtr request, MotionResultSrv::Response::SharedPtr response)
{
  INFO("Receive request once:");
  INFO("\tmotion_id: %d", request->motion_id);

  if (!IsStateValid()) {
    INFO("State invalid with current state");
    return;
  }

  decision_ptr_->Execute(request, response);
}
