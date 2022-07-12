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

namespace cyberdog
{
namespace motion
{
MotionManager::MotionManager(const std::string & name)
: manager::ManagerBase(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
}

MotionManager::~MotionManager()
{}

void MotionManager::Config()
{
  INFO("Get info from configure");
  // action_ptr_ = std::make_shared<MotionAction>();
  // handler_ptr_ = std::make_shared<MotionHandler>(motion_servo_pub_);
}

bool MotionManager::Init()
{
  INFO("Init on call");
  if (node_ptr_ == nullptr) {
    ERROR("Init failed with nullptr at ros node!");
    return false;
  }
  executor_.reset(new rclcpp::executors::MultiThreadedExecutor);
  callback_group_ = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  motion_servo_pub_ = node_ptr_->create_publisher<MotionServoResponseMsg>(
    kMotionServoResponseTopicName, 10);
  motion_status_pub_= node_ptr_->create_publisher<MotionStatusMsg>(
    kMotionStatusTopicName, 10);
  decision_ptr_ = std::make_shared<MotionDecision>();
  decision_ptr_->Init(motion_servo_pub_, motion_status_pub_);
  motion_servo_sub_ = node_ptr_->create_subscription<MotionServoCmdMsg>(
    kMotionServoCommandTopicName, rclcpp::SystemDefaultsQoS(),
    std::bind(&MotionManager::MotionServoCmdCallback, this, std::placeholders::_1));

  motion_result_srv_ =
    node_ptr_->create_service<MotionResultSrv>(
    kMotionResultServiceName,
    std::bind(
      &MotionManager::MotionResultCmdCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);


  return true;
}

void MotionManager::Run()
{
  INFO("Running on...");
  // rclcpp::spin(node_ptr_);
  executor_->add_node(node_ptr_);
  executor_->spin();
  rclcpp::shutdown();
}

bool MotionManager::SelfCheck()
{
  // check all motions from config
  return true;
}

bool MotionManager::IsStateValid()
{
  // check state from behavior tree
  return true;
}

void MotionManager::OnError()
{
  INFO("on error");
}

void MotionManager::OnLowPower()
{
  INFO("on lowpower");
}

void MotionManager::OnSuspend()
{
  INFO("on suspend");
}

void MotionManager::OnProtected()
{
  INFO("on protect");
}

void MotionManager::OnActive()
{
  INFO("on active");
}

void MotionManager::MotionServoCmdCallback(const MotionServoCmdMsg::SharedPtr msg)
{
  if (!IsStateValid()) {
    INFO("motion state invalid with current state");
    return;
  }

  decision_ptr_->DecideServoCmd(msg);
}

void MotionManager::MotionResultCmdCallback(
  const MotionResultSrv::Request::SharedPtr request, MotionResultSrv::Response::SharedPtr response)
{
  INFO("Receive request once:");
  INFO("\tmotion_id: %d", request->motion_id);

  if (!IsStateValid()) {
    INFO("State invalid with current state");
    return;
  }

  decision_ptr_->DecideResultCmd(request, response);
}
}  // namespace motion
}  // namespace cyberdog
