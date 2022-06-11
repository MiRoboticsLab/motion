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
#include "motion_manager/motion_macros.hpp"
#include "cyberdog_common/cyberdog_msg_queue.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"


namespace cyberdog
{
namespace motion
{

class MotionDecision final
{
  using MotionServoCmdMsg = protocol::msg::MotionServoCmd;
  using MotionServoResponseMsg = protocol::msg::MotionServoResponse;
  using MotionResultSrv = protocol::srv::MotionResultCmd;
  using MotionStatusMsg = protocol::msg::MotionStatus;

public:
  MotionDecision();
  ~MotionDecision();

  void Config();
  bool Init(rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub);

public:
  void DecideServoCmd(const MotionServoCmdMsg::SharedPtr msg);

  void DecideResultCmd(
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
  bool WaitHandlingResult(int32_t motion_id, int32_t duration, int32_t & code);
  // void ServoDataCheck();
  void StopMotion();
  void ServoResponseThread();

  /**
   * @brief 设置工作状态
   *        后续考虑改成Handler持有的status，并且放在macros里面
   *
   * @param status_code
   */
  inline void SetWorkStatus(DecisionStatus status_code)
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

  // /**
  //  * @brief 伺服指令检测功能开关
  //  *
  //  * @param check_flag true: 打开检测功能; false: 关闭
  //  */
  // inline void SetServoNeedCheck(bool check_flag)
  // {
  //   std::unique_lock<std::mutex> lk(servo_check_mutex_);
  //   if (check_flag && (!is_servo_need_check_)) {  // 从关闭状态打开时，重置检测数据， 并唤醒线程
  //     server_check_error_counter_ = 0;
  //     is_servo_need_check_ = check_flag;
  //     servo_check_cv_.notify_one();
  //   } else {
  //     is_servo_need_check_ = check_flag;
  //   }
  // }

  // /**
  //  * @brief 等待伺服检测开关打开
  //  *        1. 如果已经打开，则忽略；
  //  *        2. 如果已经关闭，则挂起当前线程；
  //  *
  //  */
  // inline void WaitServoNeedCheck()
  // {
  //   std::unique_lock<std::mutex> lk(servo_check_mutex_);
  //   if (!is_servo_need_check_) {
  //     servo_check_cv_.wait(lk);
  //   }
  // }

  // /**
  //  * @brief 入队一条伺服检测信号
  //  *
  //  */
  // inline void SetServoCheck()
  // {
  //   servo_check_click_->Tick();
  // }

  // /**
  //  * @brief 出队一条伺服检测信号
  //  *
  //  */
  // inline bool GetServoCheck()
  // {
  //   return servo_check_click_->Tock();
  // }

  /**
   * @brief 重置伺服反馈消息
   *
   */
  inline void ResetServoResponseMsg()
  {
    servo_response_msg_.result = true;
    servo_response_msg_.code = (int32_t)MotionCode::kOK;
  }

private:
  // std::shared_ptr<MotionAction> action_ptr_ {nullptr};
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
  // std::thread servo_data_check_thread_;
  common::MsgQueue<int> servo_response_queue_;
  // std::shared_ptr<ServoClick> servo_check_click_;
  // std::mutex servo_check_mutex_;
  // std::condition_variable servo_check_cv_;
  // bool is_servo_need_check_ {false};
  // int8_t server_check_error_counter_ {0};
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub_;
  MotionServoResponseMsg servo_response_msg_;
};  // class MotionDecision
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_DECISION_HPP_
