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
#include <memory>
#include <vector>
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_manager/motion_decision.hpp"

namespace cyberdog
{
namespace motion
{

MotionDecision::MotionDecision()
{}

MotionDecision::~MotionDecision() {}

void MotionDecision::Config() {}

bool MotionDecision::Init(const rclcpp::Node::SharedPtr node)
{
  node_ptr_ = node;
  handler_ptr_ = std::make_shared<MotionHandler>();
  if (!handler_ptr_->Init(node_ptr_)) {
    ERROR("Fail to initialize MotionHandler");
    return false;
  }
  servo_response_pub_ = node_ptr_->create_publisher<MotionServoResponseMsg>(
    kMotionServoResponseTopicName, 10);
  servo_response_thread_ = std::thread(std::bind(&MotionDecision::ServoResponseThread, this));
  servo_response_thread_.detach();
  ResetServoResponseMsg();
  return true;
}

void MotionDecision::DecideServoCmd(const MotionServoCmdMsg::SharedPtr msg)
{
  SetServoResponse();
  if (!IsStateValid(msg->motion_id)) {
    servo_response_msg_.motion_id = handler_ptr_->GetMotionStatus()->motion_id;
    servo_response_msg_.result = false;
    servo_response_msg_.code = MotionCodeMsg::TASK_STATE_ERROR;
    ERROR("Forbidden ServoCmd when estop");
    return;
  }
  handler_ptr_->HandleServoCmd(msg, servo_response_msg_);
}

/**
 * @brief 伺服指令反馈线程，运行在死循环线程中
 *        通过NeedServerResponse来挂起 / 唤醒线程
 *
 */
void MotionDecision::ServoResponseThread()
{
  while (true) {
    if (!NeedServoResponse()) {
      continue;
    }
    if (servo_response_pub_ != nullptr) {
      // FIXME(harvey): 伺服指令的运行结果判断机制
      if (!handler_ptr_->FeedbackTimeout()) {
        servo_response_msg_.motion_id = handler_ptr_->GetMotionStatus()->motion_id;
        servo_response_msg_.order_process_bar = handler_ptr_->GetMotionStatus()->order_process_bar;
        servo_response_msg_.status = handler_ptr_->GetMotionStatus()->switch_status;
        servo_response_pub_->publish(servo_response_msg_);
      } else {
        servo_response_msg_.motion_id = -1;
        servo_response_msg_.order_process_bar = -1;
        servo_response_msg_.status = -1;
        servo_response_msg_.result = false;
        servo_response_msg_.code = MotionCodeMsg::COM_LCM_TIMEOUT;
        servo_response_pub_->publish(servo_response_msg_);
      }
    }
  }
}

/**
 * @brief 执行结果指令
 *
 * @param request
 * @param response
 */
void MotionDecision::DecideResultCmd(
  const MotionResultSrv::Request::SharedPtr request,
  MotionResultSrv::Response::SharedPtr response)
{
  if (!IsStateValid(request->motion_id)) {
    response->motion_id = handler_ptr_->GetMotionStatus()->motion_id;
    response->result = false;
    response->code = MotionCodeMsg::TASK_STATE_ERROR;
    ERROR("Forbidden ResultCmd(%d) when estop", request->motion_id);
    return;
  }
  if (!IsModeValid()) {
    return;
  }
  estop_ = request->motion_id == MotionIDMsg::ESTOP;
  handler_ptr_->HandleResultCmd(request, response);
}


/**
 * @brief 执行结果指令
 *
 * @param request
 * @param response
 */
void MotionDecision::DecideQueueCmd(
  const MotionQueueCustomSrv::Request::SharedPtr request,
  MotionQueueCustomSrv::Response::SharedPtr response)
{
  if (!IsStateValid()) {
    response->result = false;
    return;
  }
  if (!IsModeValid()) {
    return;
  }
  handler_ptr_->HandleQueueCmd(request, response);
}

}  // namespace motion
}  // namespace cyberdog
