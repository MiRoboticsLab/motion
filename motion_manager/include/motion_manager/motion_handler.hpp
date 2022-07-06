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
#ifndef MOTION_MANAGER__MOTION_HANDLER_HPP_
#define MOTION_MANAGER__MOTION_HANDLER_HPP_
#include <memory>
#include <thread>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "motion_action/motion_action.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/lcm/robot_control_cmd_lcmt.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"

namespace cyberdog
{
namespace motion
{

/**
 * @brief 接收运控板返回数据，并进行解析、管理和分发
 *
 */
class MotionHandler final
{
public:
  MotionHandler();
  ~MotionHandler();

public:
  /* recv api */
  // void ServoResponse();
  bool Init();
  void UpdateMotionStatus(MotionStatusMsg::SharedPtr motion_status_ptr);
  bool CheckMotionID(int32_t motion_id);
  bool CheckMotionResult();
  bool FeedbackTimeout();
  void ServoDataCheck();
  void PoseControlDefinitively();
  void HandleServoStartFrame(const MotionServoCmdMsg::SharedPtr msg);
  void HandleServoDataFrame(const MotionServoCmdMsg::SharedPtr msg, MotionServoResponseMsg & res);
  void HandleServoCmd(const MotionServoCmdMsg::SharedPtr msg, MotionServoResponseMsg & res);
  void HandleServoEndFrame(const MotionServoCmdMsg::SharedPtr msg);
  void ExecuteResultCmd(  
  const MotionResultSrv::Request::SharedPtr request,
  MotionResultSrv::Response::SharedPtr response);
  void HandleResultCmd(
    const MotionResultSrv::Request::SharedPtr request,
    MotionResultSrv::Response::SharedPtr response);
  MotionStatusMsg::SharedPtr GetMotionStatus();
  bool CheckPreMotion(int32_t motion_id);
  bool AllowServoCmd(int32_t motion_id);
  bool isCommandValid(const MotionResultSrv::Request::SharedPtr request);
  inline void SetWorkStatus(const HandlerStatus status) {
    std::unique_lock<std::mutex> lk(status_mutex_);
    status_ = status;
  }
  inline HandlerStatus GetWorkStatus(){
    std::unique_lock<std::mutex> lk(status_mutex_);
    return status_;
  }
public:
  /* 考虑重构的API */
  bool IsIdle() {return true;}

  bool Wait4TargetMotionID(int32_t /*motion_id*/)
  {
    return true;
  }

private:
  /**
   * @brief 伺服指令检测功能开关
   *
   * @param check_flag true: 打开检测功能; false: 关闭
   */
  inline void SetServoNeedCheck(bool check_flag)
  {
    std::unique_lock<std::mutex> lk(servo_check_mutex_);
    if (check_flag && (!is_servo_need_check_)) {  // 从关闭状态打开时，重置检测数据， 并唤醒线程
      server_check_error_counter_ = 0;
      is_servo_need_check_ = check_flag;
      servo_check_cv_.notify_one();
    } else {
      is_servo_need_check_ = check_flag;
    }
  }

  /**
   * @brief 等待伺服检测开关打开
   *        1. 如果已经打开，则忽略；
   *        2. 如果已经关闭，则挂起当前线程；
   *
   */
  inline void WaitServoNeedCheck()
  {
    std::unique_lock<std::mutex> lk(servo_check_mutex_);
    if (!is_servo_need_check_) {
      servo_check_cv_.wait(lk);
    }
  }

  /**
   * @brief 入队一条伺服检测信号
   *
   */
  inline void TickServoCmd()
  {
    servo_check_click_->Tick();
  }

  /**
   * @brief 出队一条伺服检测信号
   *
   */
  inline bool TockServoCmd()
  {
    return servo_check_click_->Tock();
  }

  /* ros members */
  std::shared_ptr<MotionAction> action_ptr_ {nullptr};
  std::shared_ptr<LcmResponse> lcm_response_ {nullptr};
  std::thread servo_response_thread_;
  std::thread servo_data_check_thread_;
  std::function<void(MotionStatusMsg::SharedPtr)> motion_response_func;
  std::shared_ptr<ServoClick> servo_check_click_;
  std::mutex servo_check_mutex_;
  std::condition_variable servo_check_cv_;

  /* Execute cmd members */
  std::mutex status_mutex_;
  std::mutex feedback_mutex_;
  std::mutex execute_mutex_;
  std::condition_variable feedback_cv_;
  std::condition_variable transitioning_cv_;
  std::condition_variable execute_cv_;
  std::map<int32_t, MotionIdMap> motion_id_map_;
  MotionStatusMsg::SharedPtr motion_status_ptr_ {nullptr};
  HandlerStatus status_;
  int32_t wait_id_;
  uint8_t retry_ {0}, max_retry_{3};
  int8_t server_check_error_counter_ {0};
  bool is_transitioning_wait_ {false};
  bool is_execute_wait_ {false};
  bool is_servo_need_check_ {false};
  bool premotion_executing_ {false};
  bool pre_motion_checked_ {false};
};  // class MotionHandler
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_MANAGER__MOTION_HANDLER_HPP_
