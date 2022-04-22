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
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_manager/motion_decision.hpp"

cyberdog::motion::MotionDecision::MotionDecision(
  std::shared_ptr<MotionAction> action_ptr,
  std::shared_ptr<MotionHandler> handler_ptr)
: action_ptr_(action_ptr), handler_ptr_(handler_ptr)
{
  // action_ptr = std::make_shared<cyberdog::motion::MotionAction>();
  motion_status_ptr_ = std::make_shared<MotionStatusMsg>();
}
cyberdog::motion::MotionDecision::~MotionDecision() {}

void cyberdog::motion::MotionDecision::Config() {}

bool cyberdog::motion::MotionDecision::Init(rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub)
{
  if(handler_ptr_ == nullptr) {
    ERROR("Decision get nullptr with handler ptr!");
    return false;
  }

  servo_response_pub_ = servo_response_pub;
  servo_response_thread_ = std::thread(std::bind(&MotionDecision::ServoResponse, this));

  return true;
}
// bool cyberdog::motion::MotionDecision::CheckModeValid() {
//   return true;
// }
void cyberdog::motion::MotionDecision::Servo(const MotionServoCmdMsg::SharedPtr msg)
{
  if (!IsStateValid()) {
    return;
  }
  if (!IsModeValid()) {
    return;
  }

  switch (msg->cmd_type)
  {
    case MotionServoCmdMsg::SERVO_START:
      ServoStart(msg);
      break;
    case MotionServoCmdMsg::SERVO_DATA:
      ServoData(msg);
      break;
    case MotionServoCmdMsg::SERVO_END:
      ServoEnd(msg);
      break;
    default:
      break;
  }
}

void cyberdog::motion::MotionDecision::ServoStart(const MotionServoCmdMsg::SharedPtr msg) {
  INFO("Servo start with motion_id: %d", msg->motion_id);
  action_ptr_->Execute(msg);
  SetServoNeedResponse(false);
}

void cyberdog::motion::MotionDecision::ServoData(const MotionServoCmdMsg::SharedPtr msg) {
  action_ptr_->Execute(msg);
}

void cyberdog::motion::MotionDecision::ServoEnd(const MotionServoCmdMsg::SharedPtr msg) {
  INFO("Servo end with motion_id: %d", msg->motion_id);
  action_ptr_->Execute(msg);
  SetServoNeedResponse(true);
}

void cyberdog::motion::MotionDecision::ServoResponse()
{
  MotionServoResponseMsg msg;
  while (true)
  {
    WaitServoNeedResponse();
    if(servo_response_pub_ != nullptr) {
      msg.motion_id = motion_status_ptr_->motion_id;
      msg.order_process_bar = motion_status_ptr_->order_process_bar;
      msg.status = motion_status_ptr_->switch_status;
      msg.result = true;
      msg.code = 0;
      servo_response_pub_->publish(msg);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void cyberdog::motion::MotionDecision::Execute(
  const MotionResultSrv::Request::SharedPtr request,
  MotionResultSrv::Response::SharedPtr response)
{
  action_ptr_->Execute(request);
  // auto duration = request->duration;
  response->result = WaitExecute(request->motion_id, request->duration, response->code);
  response->motion_id = motion_status_ptr_->motion_id;
}

bool cyberdog::motion::MotionDecision::WaitExecute(int32_t motion_id, int32_t duration, int32_t & code)
{
  bool result = false;
  std::unique_lock<std::mutex> lk(execute_mutex_);
  is_execute_wait_ = true;
  wait_id = motion_id;
  auto wait_status = execute_cv_.wait_for(lk, std::chrono::milliseconds(duration));
  if(wait_status == std::cv_status::timeout) {
    code = 331;
  } else if(motion_status_ptr_->motion_id != motion_id) {
    code = 332;
  } else {
    code = 0;
    result = true;
  }
  return result;
}

void cyberdog::motion::MotionDecision::Update(MotionStatusMsg::SharedPtr motion_status_ptr)
{
  std::unique_lock<std::mutex> lk(execute_mutex_);
  motion_status_ptr_->motion_id = motion_status_ptr->motion_id;
  motion_status_ptr_->contact = motion_status_ptr->contact;
  motion_status_ptr_->order_process_bar = motion_status_ptr->order_process_bar;
  motion_status_ptr_->switch_status = motion_status_ptr->switch_status;
  motion_status_ptr_->ori_error = motion_status_ptr->ori_error;
  motion_status_ptr_->footpos_error = motion_status_ptr->footpos_error;
  for(size_t i = 0; i < motion_status_ptr->motor_error.size(); ++i) {
    motion_status_ptr_->motor_error[i] = motion_status_ptr->motor_error[i];
  }
  if(is_execute_wait_ && ((motion_status_ptr_->motion_id == wait_id) || (motion_status_ptr_->switch_status != MotionStatusMsg::NORMAL))) {
    is_execute_wait_ = false;
    execute_cv_.notify_one();
  }
}
