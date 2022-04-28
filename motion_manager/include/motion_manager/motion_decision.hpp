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
#ifndef MOTION_MANAGER__MOTION_DECISION_HPP_
#define MOTION_MANAGER__MOTION_DECISION_HPP_
#include <thread>
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>
#include "motion_action/motion_action.hpp"
#include "motion_manager/motion_handler.hpp"
#include "cyberdog_common/cyberdog_msg_queue.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"


namespace cyberdog
{
namespace motion
{

enum class DecisionStatus : uint8_t
{
  kIdle = 0,
  kServoStart = 1,
  kExecuting = 2
};

class MotionDecision final
{
  using MotionServoCmdMsg = protocol::msg::MotionServoCmd;
  using MotionServoResponseMsg = protocol::msg::MotionServoResponse;
  using MotionResultSrv = protocol::srv::MotionResultCmd;
  using MotionStatusMsg = protocol::msg::MotionStatus;

public:
  MotionDecision(
    std::shared_ptr<MotionAction> action_ptr,
    std::shared_ptr<cyberdog::motion::MotionHandler> handler_ptr);
  ~MotionDecision();

  void Config();
  bool Init(rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub);

public:
  void Servo(const MotionServoCmdMsg::SharedPtr msg);

  void Execute(
    const MotionResultSrv::Request::SharedPtr request,
    MotionResultSrv::Response::SharedPtr response);

  inline void SetMode(uint8_t mode)
  {
    motion_work_mode_ = (DecisionStatus)mode;
  }

private:
  inline bool IsStateValid()
  {
    return true;
  }

  inline bool IsModeValid()
  {
    return true;
  }

private:
  void ServoStart(const MotionServoCmdMsg::SharedPtr msg);
  void ServoData(const MotionServoCmdMsg::SharedPtr msg);
  void ServoEnd(const MotionServoCmdMsg::SharedPtr msg);
  void Update(MotionStatusMsg::SharedPtr motion_status_ptr);
  bool WaitExecute(int32_t motion_id, int32_t duration, int32_t & code);
  void ServoDataCheck();
  void StopMotion();
  void ServoResponse();

  inline void SetWorkStatus(DecisionStatus status_code)
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    motion_work_mode_ = status_code;
  }

  inline DecisionStatus GetWorkStatus()
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    return motion_work_mode_;
  }

  inline bool NeedServoResponse()
  {
    int unused_counter;
    return servo_response_queue_.DeQueue(unused_counter);
  }

  inline void SetServoResponse()
  {
    servo_response_queue_.EnQueue(1);
  }

  inline void StopServoResponse()
  {
    servo_response_queue_.Reset();
  }

  inline void SetServoNeedCheck(bool check_flag)
  {
    std::unique_lock<std::mutex> lk(servo_check_mutex_);
    if (check_flag && (!is_servo_need_check_)) {
      server_check_error_counter_ = 0;
      is_servo_need_check_ = check_flag;
      servo_check_cv_.notify_one();
    } else {
      is_servo_need_check_ = check_flag;
    }
  }

  inline void WaitServoNeedCheck()
  {
    std::unique_lock<std::mutex> lk(servo_check_mutex_);
    if (!is_servo_need_check_) {
      servo_check_cv_.wait(lk);
    }
  }

  inline void SetServoCheck()
  {
    servo_check_queue_.EnQueueOne(1);
  }

  inline bool GetServoCheck()
  {
    int unused_counter;
    if (servo_check_queue_.IsEmpty()) {
      return false;
    } else {
      servo_check_queue_.DeQueue(unused_counter);
      return true;
    }
  }

  inline void ResetServoResponseMsg()
  {
    servo_response_msg_.result = true;
    servo_response_msg_.code = 300;
  }

private:
  std::shared_ptr<MotionAction> action_ptr_ {nullptr};
  std::shared_ptr<MotionHandler> handler_ptr_ {nullptr};
  DecisionStatus motion_work_mode_ {DecisionStatus::kIdle};
  std::mutex status_mutex_;

  /* Execute cmd members */
  std::mutex execute_mutex_;
  std::condition_variable execute_cv_;
  bool is_execute_wait_ {false};
  int32_t wait_id;
  MotionStatusMsg::SharedPtr motion_status_ptr_ {nullptr};

  /* Servo cmd members */
  std::thread servo_response_thread_;
  std::thread servo_data_check_thread_;
  common::MsgQueue<int> servo_response_queue_;
  common::MsgQueue<int> servo_check_queue_;
  std::mutex servo_check_mutex_;
  std::condition_variable servo_check_cv_;
  bool is_servo_need_check_ {false};
  int8_t server_check_error_counter_ {0};
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub_;
  MotionServoResponseMsg servo_response_msg_;
};  // class MotionDecision
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_DECISION_HPP_
