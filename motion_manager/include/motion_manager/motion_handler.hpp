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
#include <sys/time.h>
#include <sys/dir.h>
#include <sys/stat.h>
#include <memory>
#include <thread>
#include <map>
#include <string>
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
  bool Init(rclcpp::Publisher<MotionStatusMsg>::SharedPtr motion_status_pub);
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
  inline void SetWorkStatus(const HandlerStatus status)
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    status_ = status;
  }
  inline HandlerStatus GetWorkStatus()
  {
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
  void WriteTomlLog(const robot_control_cmd_lcmt & cmd);
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
  inline void WaitServoNeedCheck()
  {
    std::unique_lock<std::mutex> lk(servo_check_mutex_);
    if (!is_servo_need_check_) {
      servo_check_cv_.wait(lk);
    }
  }
  inline void TickServoCmd()
  {
    servo_check_click_->Tick();
  }
  inline bool TockServoCmd()
  {
    return servo_check_click_->Tock();
  }

  std::string GetTime()
  {
    struct timeval tv;
    char buf[64];
    gettimeofday(&tv, NULL);
    strftime(buf, sizeof(buf) - 1, "%Y%m%d-%H%M%S", localtime(&tv.tv_sec));
    std::string s(buf);
    return s;
  }

  /* ros members */
  rclcpp::Publisher<MotionStatusMsg>::SharedPtr motion_status_pub_ {nullptr};
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
  std::ofstream toml_;
  std::string toml_log_dir_;
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
