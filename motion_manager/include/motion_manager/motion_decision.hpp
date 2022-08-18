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
#include <vector>
#include "motion_action/motion_macros.hpp"
#include "motion_action/motion_action.hpp"
#include "motion_manager/motion_handler.hpp"
#include "cyberdog_common/cyberdog_msg_queue.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "motion_utils/motion_utils.hpp"

namespace cyberdog
{
namespace motion
{

class MotionDecision final
{
public:
  MotionDecision(const rclcpp::Node::SharedPtr & node, const std::shared_ptr<MCode> & code);
  ~MotionDecision();

  void Config();
  bool Init();

public:
  void DecideServoCmd(const MotionServoCmdMsg::SharedPtr & msg);
  void DecideResultCmd(
    const MotionResultSrv::Request::SharedPtr request,
    MotionResultSrv::Response::SharedPtr response);
  void DecideQueueCmd(
    const MotionQueueCustomSrv::Request::SharedPtr request,
    MotionQueueCustomSrv::Response::SharedPtr response);

  inline void SetMode(uint8_t mode)
  {
    motion_work_mode_ = (DecisionStatus)mode;
  }

private:
  inline bool IsStateValid(int32_t motion_id)
  {
    if (estop_) {
      return motion_id == MotionIDMsg::ESTOP ||
             motion_id == MotionIDMsg::RECOVERYSTAND;
    }
    return true;
  }
  inline bool IsStateValid()
  {
    return estop_;
  }

  inline bool IsModeValid()
  {
    return true;
  }

private:
  void ServoStart(const MotionServoCmdMsg::SharedPtr & msg);
  void ServoData(const MotionServoCmdMsg::SharedPtr & msg);
  void ServoEnd(const MotionServoCmdMsg::SharedPtr & msg);
  void Update(MotionStatusMsg::SharedPtr & motion_status_ptr);
  bool WaitHandlingResult(int32_t motion_id, int32_t duration, int32_t & code);
  void StopMotion();
  void ServoResponseThread();
  inline void SetWorkStatus(const DecisionStatus & status_code)
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    motion_work_mode_ = status_code;
  }

  /**
   * @brief Get the Work Status object
   *
   * @return DecisionStatus
   */
  inline DecisionStatus GetWorkStatus()
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    return motion_work_mode_;
  }

  /**
   * @brief 消费一条伺服反馈计数
   *
   * @return true 消费正常
   * @return false 消费异常，此时消息队列析构或reset，需要注意后续代码安全
   */
  inline bool NeedServoResponse()
  {
    int unused_counter;
    return servo_response_queue_.DeQueue(unused_counter);
  }

  /**
   * @brief 生产一条伺服反馈计数
   *
   */
  inline void SetServoResponse()
  {
    servo_response_queue_.EnQueue(1);
  }

  /**
   * @brief 重置伺服反馈计数队列
   *        该行为会导致计数消费函数退出且返回false
   *
   */
  inline void ResetServoResponse()
  {
    servo_response_queue_.Reset();
  }

  /**
   * @brief 重置伺服反馈消息
   *
   */
  inline void ResetServoResponseMsg()
  {
    servo_response_msg_.result = true;
    servo_response_msg_.code = MotionCodeMsg::OK;
  }

private:
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  std::shared_ptr<MotionHandler> handler_ptr_ {nullptr};
  DecisionStatus motion_work_mode_ {DecisionStatus::kIdle};
  std::mutex status_mutex_;
  std::shared_ptr<MCode> code_ptr_;

  /* Servo cmd members */
  std::thread servo_response_thread_;
  common::MsgQueue<int> servo_response_queue_;
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub_;
  MotionServoResponseMsg servo_response_msg_;
  std::shared_ptr<LaserHelper> laser_helper_;
  bool estop_ {false};
};  // class MotionDecision
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_DECISION_HPP_
