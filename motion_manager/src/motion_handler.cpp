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
#include "motion_manager/motion_handler.hpp"

cyberdog::motion::MotionHandler::MotionHandler()
{
  // (void) publisher;
  // servo_data_check_thread_ = std::thread(std::bind(&MotionHandler::ServoDataCheck, this));
}

cyberdog::motion::MotionHandler::~MotionHandler()
{}

void cyberdog::motion::MotionHandler::Update()
{}

bool cyberdog::motion::MotionHandler::Init()
{
  action_ptr_ = std::make_shared<MotionAction>();
  if(!action_ptr_->Init()) {
    ERROR("Fail to initialize MotionAction");
    return false;
  }
  servo_check_click_ = std::make_shared<ServoClick>();
  servo_data_check_thread_ = std::thread(std::bind(&MotionHandler::ServoDataCheck, this));
  // action_ptr_->RegisterFeedback(
  //   std::bind(&MotionHandler::Checkout, this,std::placeholders::_1));
  return true;
}
void cyberdog::motion::MotionHandler::RegisterUpdate(std::function<void(MotionStatusMsg::SharedPtr)> f)
{
  action_ptr_->RegisterFeedback(f);
}
// void cyberdog::motion::MotionHandler::Checkout(MotionStatusMsg::SharedPtr motion_status_ptr)
// {
//   motion_response_func(motion_status_ptr);
// }

void cyberdog::motion::MotionHandler::HandleServoStartFrame(const MotionServoCmdMsg::SharedPtr msg)
{
  action_ptr_->Execute(msg);
  TickServoCmd();
  SetServoNeedCheck(true);
}

void cyberdog::motion::MotionHandler::HandleServoDataFrame(const MotionServoCmdMsg::SharedPtr msg)
{
  action_ptr_->Execute(msg);
  TickServoCmd();
}

void cyberdog::motion::MotionHandler::HandleServoEndFrame(const MotionServoCmdMsg::SharedPtr msg)
{
  action_ptr_->Execute(msg);
  SetServoNeedCheck(false);
}

/**
 * @brief 检测伺服指令的下发间隔是否符合要求
 *        1. 运行在死循环线程中，通过waitServoNeedCheck进行线程挂起与唤醒操作
 *
 */
void cyberdog::motion::MotionHandler::ServoDataCheck()
{
  while (true) {
    WaitServoNeedCheck();
    if (!TockServoCmd()) {
      server_check_error_counter_++;
    } else {
      server_check_error_counter_ = 0;
    }
    if (server_check_error_counter_ >= 4) {
      WARN("Servo data lost time with 4 times");
      // StopServoResponse();
      // SetServoDataLost(); TODO 是否通知Decision？
      StandBy();
      SetServoNeedCheck(false);
      // SetWorkStatus(DecisionStatus::kIdle); TODO 当信号丢失的时候，是否需要将Decision中的状态复位？
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

/**
 * @brief 停止运动，让机器人回归站立姿态
 *        后续计划改成调用当前动作的状态机结束动作，而不是写死
 *
 */
void cyberdog::motion::MotionHandler::StandBy()
{
  MotionResultSrv::Request::SharedPtr request(new MotionResultSrv::Request);
  request->motion_id = 1;
  request->pos_des = std::vector<float>{0.0, 0.0, 0.3};
  action_ptr_->Execute(request);
}

/**
 * @brief 执行结果指令
 *
 * @param request
 * @param response
 */
void cyberdog::motion::MotionHandler::HandleResultCmd(
  const MotionResultSrv::Request::SharedPtr request,
  MotionResultSrv::Response::SharedPtr response)
{
  action_ptr_->Execute(request);
  // auto duration = request->duration;
  // response->result = WaitExecute(request->motion_id, request->duration, response->code);
  // response->motion_id = motion_status_ptr_->motion_id;
}


// void cyberdog::motion::MotionHandler::ServoResponse()
// {

// }
