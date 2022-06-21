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
{
  // action_ptr = std::make_shared<MotionAction>();
  // motion_status_ptr_ = std::make_shared<MotionStatusMsg>();
  // motion_status_ptr_->motor_error.resize(12);
  // servo_check_click_ = std::make_shared<ServoClick>();
  // ResetServoResponseMsg();
}
MotionDecision::~MotionDecision() {}

void MotionDecision::Config() {}

bool MotionDecision::Init(
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub)
{
  handler_ptr_ = std::make_shared<MotionHandler>();
  if (!handler_ptr_->Init()) {
    ERROR("Fail to initialize MotionHandler");
    return false;
  }
  servo_response_pub_ = servo_response_pub;
  servo_response_thread_ = std::thread(std::bind(&MotionDecision::ServoResponseThread, this));
  ResetServoResponseMsg();
  return true;
}

bool MotionDecision::AllowServoCmd()
{
  // TODO: 判断当前状态是否能够行走
  return handler_ptr_->GetMotionStatus()->motion_id == 1 ||
    handler_ptr_->GetMotionStatus()->motion_id ==12; 
}

/**
 * @brief 伺服指令接收入口
 *        重构思路：
 *          1. servo数据装入buffer， 进行平滑缓冲
 *          2. 产生类似ur机械臂的servo控制接口
 *
 * @param msg
 */
void MotionDecision::DecideServoCmd(const MotionServoCmdMsg::SharedPtr msg)
{
  SetServoResponse();
  if (!IsStateValid()) {
    servo_response_msg_.result = false;
    servo_response_msg_.code = (int32_t)MotionCode::kStateError;
    return;
  }

  if (msg->cmd_type != MotionServoCmdMsg::SERVO_END){
    if(!AllowServoCmd()) {
      if (retry_ < max_retry_) {
        MotionResultSrv::Request::SharedPtr request(new MotionResultSrv::Request);
        MotionResultSrv::Response::SharedPtr response(new MotionResultSrv::Response);
        request->motion_id = 1;
        INFO("Trying to be ready for ServoCmd");
        handler_ptr_->HandleResultCmd(request, response);
        if (!response->result) {
          retry_++;
        } else {
          retry_ = 0;
        }
      } else {
        servo_response_msg_.result = false;
        servo_response_msg_.code = (int32_t)MotionCode::kSwitchError;
      }
      return;
    }
    handler_ptr_->HandleServoDataFrame(msg);
  }
  else {
    handler_ptr_->HandleServoEndFrame(msg);
    ResetServoResponse();
  }
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
      servo_response_msg_.motion_id = handler_ptr_->GetMotionStatus()->motion_id;
      servo_response_msg_.order_process_bar = handler_ptr_->GetMotionStatus()->order_process_bar;
      servo_response_msg_.status = handler_ptr_->GetMotionStatus()->switch_status;
      servo_response_pub_->publish(servo_response_msg_);
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
  if (!IsStateValid()) {
    response->motion_id = -1;
    response->result = false;
    response->code = (int32_t)MotionCode::kStateError;
    return;
  }
  if (!IsModeValid()) {
    return;
  }
  handler_ptr_->HandleResultCmd(request, response);
}
}  // namespace motion
}  // namespace cyberdog
