// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <algorithm>
#include <vector>
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
  MotionHandler(const rclcpp::Node::SharedPtr & node, const std::shared_ptr<MCode> & code);
  ~MotionHandler();

public:
  bool Init();
  void HandleServoCmd(const MotionServoCmdMsg::SharedPtr & msg, MotionServoResponseMsg & res);
  template<typename CmdRequestT, typename CmdResponseT>
  void ExecuteResultCmd(const CmdRequestT request, CmdResponseT response);
  template<typename CmdRequestT, typename CmdResponseT>
  void HandleResultCmd(const CmdRequestT request, CmdResponseT response);
  // void HandleSequenceCmd(
  //   const MotionSequenceSrv::Request::SharedPtr request,
  //   MotionSequenceSrv::Response::SharedPtr response);
  void HandleQueueCmd(
    const MotionQueueCustomSrv::Request::SharedPtr request,
    MotionQueueCustomSrv::Response::SharedPtr response);
  MotionStatusMsg::SharedPtr GetMotionStatus();
  bool FeedbackTimeout();
  // inline void SetSequnceTotalDuration(int64_t sequence_total_duration)
  // {
  //   sequence_total_duration_ = sequence_total_duration;
  // }
  bool CheckMotionResult(int32_t & code);
  bool CheckMotionResult(int32_t motion_id, int32_t & code);
  bool SelfCheck()
  {
    return action_ptr_->SelfCheck();
  }
  void SetState(const MotionMgrState & state)
  {
    fsm_state_ = state;
    action_ptr_->SetState(state);
  }
  void RegisterModeFunction(std::function<void()> function)
  {
    reset_decision_f_ = function;
  }

private:
  void UpdateMotionStatus(const MotionStatusMsg::SharedPtr & motion_status_ptr);
  bool CheckMotionID(int32_t motion_id);
  void ServoDataCheck();
  void PoseControlDefinitively();
  void WalkStand(const MotionServoCmdMsg::SharedPtr & last_servo_cmd);
  void HandleServoStartFrame(const MotionServoCmdMsg::SharedPtr & msg);
  void HandleServoDataFrame(const MotionServoCmdMsg::SharedPtr & msg, MotionServoResponseMsg & res);
  void HandleServoEndFrame(const MotionServoCmdMsg::SharedPtr & msg);
  bool CheckPostMotion(int32_t motion_id);
  bool AllowServoCmd(int32_t motion_id);
  template<typename CmdRequestT>
  bool IsCommandValid(const CmdRequestT & request, int32_t & code);
  bool CheckMotors(int32_t & code);
  inline void SetWorkStatus(const HandlerStatus & status)
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    status_ = status;
  }
  inline HandlerStatus GetWorkStatus()
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    return status_;
  }
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
  void StopServoCmd();
  inline std::string GetCurrentTime()
  {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    static const int MAX_BUFFER_SIZE = 128;
    char timestamp_str[MAX_BUFFER_SIZE];
    time_t sec = static_cast<time_t>(tv.tv_sec);
    int ms = static_cast<int>(tv.tv_usec) / 1000;
    struct tm tm_time;
    localtime_r(&sec, &tm_time);
    static const char * formater = "%4d_%02d_%02d-%02d:%02d:%02d.%03d";
    int wsize = snprintf(
      timestamp_str, MAX_BUFFER_SIZE, formater,
      tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
      tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec, ms);
    timestamp_str[std::min(wsize, MAX_BUFFER_SIZE - 1)] = '\0';
    return std::string(timestamp_str);
  }

  inline void CreateTomlLog(int32_t motion_id)
  {
    toml_.open(
      getenv("HOME") + std::string("/TomlLog/") + GetCurrentTime() +
      "-" + std::to_string(motion_id) + ".toml");
    toml_.setf(std::ios::fixed, std::ios::floatfield);
    toml_.precision(3);
  }

  inline void CreateTomlLog(const std::string motion_id)
  {
    toml_.open(
      getenv("HOME") + std::string("/TomlLog/") + GetCurrentTime() +
      "-" + motion_id + ".toml");
    toml_.setf(std::ios::fixed, std::ios::floatfield);
    toml_.precision(3);
  }

  inline void CloseTomlLog()
  {
    toml_.close();
  }

  /* ros members */
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::Publisher<MotionStatusMsg>::SharedPtr motion_status_pub_ {nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr ad_srv_{nullptr};
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
  MotionServoCmdMsg::SharedPtr last_servo_cmd_ {nullptr};
  MotionStatusMsg::SharedPtr motion_status_ptr_ {nullptr};
  MotionMgrState fsm_state_;
  HandlerStatus status_;
  std::ofstream toml_;
  std::shared_ptr<MCode> code_ptr_;
  std::string toml_log_dir_;
  std::function<void()> reset_decision_f_;
  // int64_t sequence_total_duration_{0};
  int32_t wait_id_;
  uint8_t retry_ {0}, max_retry_{3};
  int8_t server_check_error_counter_ {0};
  bool is_transitioning_wait_ {false};
  bool is_execute_wait_ {false};
  bool is_servo_need_check_ {false};
  bool premotion_executing_ {false};
  bool post_motion_checked_ {false};
  bool exec_servo_pre_motion_failed_ {false};
};  // class MotionHandler
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_MANAGER__MOTION_HANDLER_HPP_
