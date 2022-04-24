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
  kServoStart = 0,
  KServoData = 1,
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
    motion_control_mode_ = mode;
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

  void StopMotion();
  void ServoResponse();
  inline void WaitServoNeedResponse() {
    std::unique_lock<std::mutex> lk(servo_mutex_);
    if(is_servo_wait_) {
      servo_cv_.wait(lk);
    }
  }
  inline void SetServoNeedResponse(bool wait_flag) {
    std::unique_lock<std::mutex> lk(servo_mutex_);
    if(is_servo_wait_ && ! wait_flag) {
      is_servo_wait_ = wait_flag;
      servo_cv_.notify_one();
    } else {
      is_servo_wait_ = wait_flag;
    }
  }

private:
  std::shared_ptr<MotionAction> action_ptr_ {nullptr};
  std::shared_ptr<MotionHandler> handler_ptr_ {nullptr};
  uint8_t motion_control_mode_ {0};
  std::mutex execute_mutex_;
  std::condition_variable execute_cv_;
  bool is_execute_wait_ {false};
  int32_t wait_id;
  MotionStatusMsg::SharedPtr motion_status_ptr_ {nullptr};
  std::thread servo_response_thread_;
  std::mutex servo_mutex_;
  std::condition_variable servo_cv_;
  bool is_servo_wait_ {true};
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub_;
};  // class MotionDecision
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_DECISION_HPP_
