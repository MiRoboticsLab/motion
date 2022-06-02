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
  motion_status_ptr_->motor_error.resize(12);
  servo_check_click_ = std::make_shared<ServoClick>();
  ResetServoResponseMsg();
}
cyberdog::motion::MotionDecision::~MotionDecision() {}

void cyberdog::motion::MotionDecision::Config() {}

bool cyberdog::motion::MotionDecision::Init(
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub)
{
  if (handler_ptr_ == nullptr) {
    ERROR("Decision get nullptr with handler ptr!");
    return false;
  }

  handler_ptr_->RegisterUpdate(std::bind(&MotionDecision::Update, this, std::placeholders::_1));

  servo_response_pub_ = servo_response_pub;
  servo_response_thread_ = std::thread(std::bind(&MotionDecision::ServoResponse, this));

  return true;
}

/**
 * @brief 伺服指令接收入口
 *        重构思路：
 *          1. servo数据装入buffer， 进行平滑缓冲
 *          2. 产生类似ur机械臂的servo控制接口
 *
 * @param msg
 */
void cyberdog::motion::MotionDecision::Servo(const MotionServoCmdMsg::SharedPtr msg)
{
  // 后续应该只检测运动状态，不检测mode
  if (!IsStateValid()) {
    return;
  }
  if (!IsModeValid()) {
    return;
  }

  switch (msg->cmd_type) {
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

void cyberdog::motion::MotionDecision::ServoStart(const MotionServoCmdMsg::SharedPtr msg)
{
  INFO("Servo start with motion_id: %d", msg->motion_id);
  action_ptr_->Execute(msg);
  ResetServoResponseMsg();
  SetWorkStatus(DecisionStatus::kServoStart);
  SetServoResponse();
  SetServoNeedCheck(true);
}

void cyberdog::motion::MotionDecision::ServoData(const MotionServoCmdMsg::SharedPtr msg)
{
  if (DecisionStatus::kServoStart != GetWorkStatus()) {
    servo_response_msg_.result = false;
    servo_response_msg_.code = (int32_t)MotionCode::kCheckError;
    return;
  }
  action_ptr_->Execute(msg);
  SetServoResponse();
}

void cyberdog::motion::MotionDecision::ServoEnd(const MotionServoCmdMsg::SharedPtr msg)
{
  INFO("Servo end with motion_id: %d", msg->motion_id);
  action_ptr_->Execute(msg);
  StopServoResponse();
  StopMotion();
  SetServoNeedCheck(false);
  SetWorkStatus(DecisionStatus::kIdle);
}

/**
 * @brief 私服指令反馈线程，运行在死循环线程中
 *        通过NeedServerResponse来挂起 / 唤醒线程
 *
 */
void cyberdog::motion::MotionDecision::ServoResponse()
{
  while (true) {
    if (!NeedServoResponse()) {
      continue;
    }
    if (servo_response_pub_ != nullptr) {
      servo_response_msg_.motion_id = motion_status_ptr_->motion_id;
      servo_response_msg_.order_process_bar = motion_status_ptr_->order_process_bar;
      servo_response_msg_.status = motion_status_ptr_->switch_status;
      // servo_response_msg.result = true;
      // servo_response_msg.code = 0;
      servo_response_pub_->publish(servo_response_msg_);
    }
  }
}

/**
 * @brief 检测私服指令的下发间隔是否符合要求
 *        1. 运行在死循环线程中，通过waitServoNeedCheck进行线程挂起与唤醒操作
 *
 */
void cyberdog::motion::MotionDecision::ServoDataCheck()
{
  while (true) {
    WaitServoNeedCheck();
    if (!GetServoCheck()) {
      server_check_error_counter_++;
    }
    if (server_check_error_counter_ >= 4) {
      WARN("Servo data lost time with 4 times");
      StopServoResponse();
      StopMotion();
      SetServoNeedCheck(false);
      SetWorkStatus(DecisionStatus::kIdle);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

/**
 * @brief 停止运动，让机器人回归站立姿态
 *        后续计划改成调用当前动作的状态机结束动作，而不是写死
 *
 */
void cyberdog::motion::MotionDecision::StopMotion()
{
  MotionResultSrv::Request::SharedPtr request(new MotionResultSrv::Request);
  request->motion_id = 1;
  action_ptr_->Execute(request);
}

/**
 * @brief 执行结果指令
 *
 * @param request
 * @param response
 */
void cyberdog::motion::MotionDecision::Execute(
  const MotionResultSrv::Request::SharedPtr request,
  MotionResultSrv::Response::SharedPtr response)
{
  action_ptr_->Execute(request);
  // auto duration = request->duration;
  response->result = WaitExecute(request->motion_id, request->duration, response->code);
  response->motion_id = motion_status_ptr_->motion_id;
}

/**
 * @brief 结果指令的执行等待函数
 *
 * @param motion_id 目标id
 * @param duration 等待期限
 * @param code 结果编码，引用形式返回
 * @return true
 * @return false
 */
bool cyberdog::motion::MotionDecision::WaitExecute(
  int32_t motion_id, int32_t duration,
  int32_t & code)
{
  bool result = false;
  std::unique_lock<std::mutex> lk(execute_mutex_);
  is_execute_wait_ = true;
  wait_id = motion_id;
  auto wait_status = execute_cv_.wait_for(lk, std::chrono::milliseconds(duration));
  if (wait_status == std::cv_status::timeout) {
    if (motion_status_ptr_->motion_id != motion_id) {
      code = (int32_t)MotionCode::kTimeout;
    }
  } else if (motion_status_ptr_->motion_id != motion_id) {
    code = (int32_t)MotionCode::kCheckError;
  } else {
    code = (int32_t)MotionCode::kOK;
    result = true;
  }
  return result;
}

/**
 * @brief Inelegant code
 *
 * @param motion_status_ptr
 */
void cyberdog::motion::MotionDecision::Update(MotionStatusMsg::SharedPtr motion_status_ptr)
{
  std::unique_lock<std::mutex> lk(execute_mutex_);
  motion_status_ptr_->motion_id = motion_status_ptr->motion_id;
  motion_status_ptr_->contact = motion_status_ptr->contact;
  motion_status_ptr_->order_process_bar = motion_status_ptr->order_process_bar;
  motion_status_ptr_->switch_status = motion_status_ptr->switch_status;
  motion_status_ptr_->ori_error = motion_status_ptr->ori_error;
  motion_status_ptr_->footpos_error = motion_status_ptr->footpos_error;
  for (size_t i = 0; i < motion_status_ptr->motor_error.size(); ++i) {
    motion_status_ptr_->motor_error[i] = motion_status_ptr->motor_error[i];
  }
  if (is_execute_wait_ &&
    ((motion_status_ptr_->motion_id == wait_id) ||
    (motion_status_ptr_->switch_status != MotionStatusMsg::NORMAL)))
  {
    is_execute_wait_ = false;
    execute_cv_.notify_one();
  }
}
