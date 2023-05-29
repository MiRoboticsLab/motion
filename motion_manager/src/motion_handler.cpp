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
#include <memory>
#include <vector>
#include <set>
#include <string>
#include "motion_manager/motion_handler.hpp"

namespace cyberdog
{
namespace motion
{

MotionHandler::MotionHandler(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MCode> & code)
: node_ptr_(node), code_ptr_(code)
{}

MotionHandler::~MotionHandler()
{}

bool MotionHandler::Init()
{
  motion_status_ptr_ = std::make_shared<MotionStatusMsg>();
  motion_status_ptr_->motor_error.resize(12);
  action_ptr_ = std::make_shared<MotionAction>();
  action_ptr_->RegisterFeedback(
    std::bind(&MotionHandler::UpdateMotionStatus, this, std::placeholders::_1));
  action_ptr_->RegisterTomlLog(
    std::bind(&MotionHandler::WriteTomlLog, this, std::placeholders::_1));
  if (!action_ptr_->Init()) {
    ERROR("Fail to initialize MotionAction");
    return false;
  }
  motion_status_pub_ = node_ptr_->create_publisher<MotionStatusMsg>(
    kMotionStatusTopicName, 10);
  ad_srv_ = node_ptr_->create_service<std_srvs::srv::SetBool>(
    "ad",
    [this](const std_srvs::srv::SetBool_Request::SharedPtr request,
    std_srvs::srv::SetBool_Response::SharedPtr) {
      this->action_ptr_->ShowDebugLog(request->data);
    }
  );
  audio_play_ = node_ptr_->create_client<protocol::srv::AudioTextPlay>(
    "speech_text_play",
    rmw_qos_profile_services_default);
  servo_check_click_ = std::make_shared<ServoClick>();
  servo_data_check_thread_ = std::thread(std::bind(&MotionHandler::ServoDataCheck, this));
  servo_data_check_thread_.detach();
  toml_log_dir_ = getenv("HOME") + std::string("/TomlLog/");
  if (access(toml_log_dir_.c_str(), 0) != 0) {
    if (mkdir(toml_log_dir_.c_str(), 0777) != 0) {
      INFO("Cannot create TomlLog directory");
    }
  }
  motion_id_map_ = action_ptr_->GetMotionIdMap();
  SetDanceMap();
  std::thread{
    [this]() {
      while (rclcpp::ok()) {
        motion_status_pub_->publish(*motion_status_ptr_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }.detach();
  SetWorkStatus(HandlerStatus::kIdle);
  return true;
}

template<>
bool MotionHandler::IsCommandValid(const MotionServoCmdMsg::SharedPtr & request, int32_t & code)
{
  if (motion_id_map_.find(request->motion_id) == motion_id_map_.end()) {
    ERROR("Command %d not support", request->motion_id);
    code = code_ptr_->GetKeyCode(system::KeyCode::kUnSupport);
    return false;
  }
  return true;
}

template<typename CmdRequestT>
bool MotionHandler::IsCommandValid(const CmdRequestT & request, int32_t & code)
{
  if (request->motion_id != MotionIDMsg::SEQUENCE_CUSTOM) {
    if (motion_id_map_.find(request->motion_id) == motion_id_map_.end()) {
      ERROR("Command %d not support", request->motion_id);
      code = code_ptr_->GetKeyCode(system::KeyCode::kUnSupport);
      return false;
    }
    bool result = true;
    auto min_exec_time = motion_id_map_[request->motion_id].min_exec_time;
    if (min_exec_time > 0) {
      result = request->duration == 0;
    } else if (min_exec_time < 0) {
      result = request->duration > 0;
    } else {}
    if (!result) {
      ERROR("Command %d not valid", request->motion_id);
      code = code_ptr_->GetKeyCode(system::KeyCode::kParametersInvalid);
      return false;
    }
    return true;
  } else {
    // TODO(Harvey): 判断自定义动作的指令有效
    return true;
  }
}

void MotionHandler::HandleServoStartFrame(const MotionServoCmdMsg::SharedPtr & msg)
{
  action_ptr_->Execute(msg);
  TickServoCmd();
  SetServoNeedCheck(true);
}

// void MotionHandler::HandleServoDataFrame(
//   const MotionServoCmdMsg::SharedPtr & msg,
//   MotionServoResponseMsg & res)
// {
//   if (!AllowServoCmd(msg->motion_id)) {
//     if (retry_ < max_retry_) {
//       MotionResultSrv::Request::SharedPtr request(new MotionResultSrv::Request);
//       MotionResultSrv::Response::SharedPtr response(new MotionResultSrv::Response);
//       request->motion_id = MotionIDMsg::RECOVERYSTAND;
//       INFO("Trying to be ready for ServoCmd");
//       HandleResultCmd(request, response);
//       if (!response->result) {
//         retry_++;
//       } else {
//         retry_ = 0;
//       }
//     } else {
//       res.result = false;
//       res.code = code_ptr_->GetCode(MotionCode::kMotionSwitchError);
//     }
//     return;
//   }

//   action_ptr_->Execute(msg);
//   TickServoCmd();
//   SetServoNeedCheck(true);
// }

// void MotionHandler::HandleServoEndFrame(const MotionServoCmdMsg::SharedPtr & msg)
// {
//   // action_ptr_->Execute(msg);
//   (void) msg;
//   WalkStand(msg);
//   SetServoNeedCheck(false);
// }

void MotionHandler::HandleServoCmd(
  const MotionServoCmdMsg::SharedPtr & msg,
  MotionServoResponseMsg & res)
{
  if (GetWorkStatus() == HandlerStatus::kExecutingResultCmd) {
    res.result = false;
    // res.code = code_ptr_->GetCode(MotionCode::kBusy);
    res.code = code_ptr_->GetKeyCode(system::KeyCode::kTargetBusy);
    ERROR("Busy(Executing ResultCmd) for ServoCmd");
    if (reset_decision_f_ != nullptr) {
      reset_decision_f_();
    }
    return;
  }
  int32_t code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
  if (!IsCommandValid(msg, code)) {
    res.result = false;
    // res.code = code_ptr_->GetCode(MotionCode::kBusy);
    res.code = code;
    if (reset_decision_f_ != nullptr) {
      reset_decision_f_();
    }
    return;
  }
  SetWorkStatus(HandlerStatus::kExecutingServoCmd);
  if (msg->cmd_type != MotionServoCmdMsg::SERVO_END) {
    if (!AllowServoCmd(msg->motion_id)) {
      SetServoNeedCheck(false);
      MotionResultSrv::Request::SharedPtr request(new MotionResultSrv::Request);
      MotionResultSrv::Response::SharedPtr response(new MotionResultSrv::Response);
      request->motion_id = MotionIDMsg::RECOVERYSTAND;
      INFO("Trying to be ready for ServoCmd");
      ExecuteResultCmd(request, response);
      if (!response->result) {
        SetWorkStatus(HandlerStatus::kIdle);
        res.result = false;
        res.code = response->code;
        exec_servo_pre_motion_failed_ = true;
        ERROR("Get error when trying to be ready for ServoCmd");
        if (reset_decision_f_ != nullptr) {
          reset_decision_f_();
        }
        return;
      }
      post_motion_checked_ = true;
    }
    exec_servo_pre_motion_failed_ = false;
    last_servo_cmd_ = msg;
    // action_ptr_->SetAlignContact(true);
    action_ptr_->Execute(msg);
    // if (!sing_) {
    //   // Sing(true);
    //   sing_ = true;
    // }
    TickServoCmd();
    SetServoNeedCheck(true);
  } else {
    StopServoCmd();
  }
  res.result = true;
  // res.code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
}

void MotionHandler::ServoDataCheck()
{
  while (true) {
    WaitServoNeedCheck();
    if (!TockServoCmd()) {
      server_check_error_counter_++;
    } else {
      server_check_error_counter_ = 0;
    }
    if (server_check_error_counter_ >= kServoDataLostTimesThreshold) {
      WARN("Servo data lost time with %d times", kServoDataLostTimesThreshold);
      StopServoCmd();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void MotionHandler::StopServoCmd()
{
  SetServoNeedCheck(false);
  if (last_servo_cmd_ != nullptr && last_servo_cmd_->motion_id !=
    MotionIDMsg::FORCECONTROL_DEFINITIVELY)
  {
    if (fsm_state_ != MotionMgrState::kTearDown && fsm_state_ != MotionMgrState::kOTA) {
      WalkStand(last_servo_cmd_);
    } else {
      INFO("Won't walk stand due to fsm in TearDown or OTA");
    }
  }
  SetWorkStatus(HandlerStatus::kIdle);
  if (reset_decision_f_ != nullptr) {
    reset_decision_f_();
  }
  post_motion_checked_ = false;
}

void MotionHandler::PoseControlDefinitively()
{
  MotionResultSrv::Request::SharedPtr request(new MotionResultSrv::Request);
  MotionResultSrv::Response::SharedPtr response(new MotionResultSrv::Response);
  request->motion_id = MotionIDMsg::POSECONTROL_DEFINITIVELY;
  request->pos_des = std::vector<float>{0.0, 0.0, 0.225};
  request->duration = 200;
  // action_ptr_->Execute(request);
  ExecuteResultCmd(request, response);
}

void MotionHandler::WalkStand(const MotionServoCmdMsg::SharedPtr & last_servo_cmd)
{
  if (exec_servo_pre_motion_failed_) {
    WARN("===Will not WalkStand===");
    return;
  }
  MotionResultSrv::Request::SharedPtr request(new MotionResultSrv::Request);
  // request->motion_id = last_servo_cmd->motion_id;
  // request->step_height = last_servo_cmd->step_height;
  request->value = last_servo_cmd->value;
  // request->duration = 500;
  request->motion_id = MotionIDMsg::WALK_STAND;
  request->step_height = last_servo_cmd->step_height;
  CreateTomlLog(request->motion_id);
  action_ptr_->Execute(request);
  // request->motion_id = MotionIDMsg::WALK_STAND;
  // action_ptr_->Execute(request);
  CloseTomlLog();
  // ExecuteResultCmd(request, response);
}

bool MotionHandler::CheckMotionResult(int32_t & code)
{
  if (!CheckMotors(code)) {
    return false;
  }
  if (motion_status_ptr_->ori_error != 0 ||
    // TODO(harvey): footpos_error需要等到运控组确定策略后再加进来
    //  motion_status_ptr_->footpos_error == 0 &&
    (motion_status_ptr_->switch_status != MotionStatusMsg::NORMAL &&
    motion_status_ptr_->switch_status != MotionStatusMsg::TRANSITIONING))
  {
    ERROR("Motion ori error or switch error");
    code = code_ptr_->GetCode(MotionCode::kMotionExecuteError);
    return false;
  }
  return true;
}

bool MotionHandler::CheckMotionResult(int32_t motion_id, int32_t & code)
{
  if (motion_id == MotionIDMsg::ESTOP) {
    return true;
  }
  return CheckMotionResult(code);
}

bool MotionHandler::FeedbackTimeout()
{
  std::unique_lock<std::mutex> feedback_lk(feedback_mutex_);
  return feedback_cv_.wait_for(feedback_lk, std::chrono::milliseconds(kAcitonLcmReadTimeout)) ==
         std::cv_status::timeout;
}

template<typename CmdRequestT, typename CmdResponseT>
void MotionHandler::ExecuteResultCmd(const CmdRequestT request, CmdResponseT response)
{
  // if (request->motion_id != MotionIDMsg::ESTOP) {
  //   for (auto motor : motion_status_ptr_->motor_error) {
  //     if (motor != 0 && motor != kMotorNormal) {
  //       response->result = false;
  //       response->code = code_ptr_->GetCode(MotionCode::kHwMotorOffline);
  //       ERROR("Motor error");
  //       return;
  //     }
  //   }
  // }
  // if (!CheckPostMotion(request->motion_id)) {
  //   MotionResultSrv::Request::SharedPtr req(new MotionResultSrv::Request);
  //   MotionResultSrv::Response::SharedPtr res(new MotionResultSrv::Response);
  //   req->motion_id = MotionIDMsg::RECOVERYSTAND;
  //   INFO("Trying to be ready for ResultCmd");
  //   ExecuteResultCmd(req, res);
  //   if (!res->result) {
  //     response->code = res->code;
  //     response->result = false;
  //     response->motion_id = motion_status_ptr_->motion_id;
  //     ERROR("Get error when trying to be ready for ResultCmd");
  //     return;
  //   }
  // }
  action_ptr_->Execute(request);
  INFO("Wait 15ms start");
  if (request->motion_id == MotionIDMsg::SEQUENCE_CUSTOM) {
    auto req = std::make_shared<MotionResultSrv::Request>();
    req->motion_id = request->motion_id;
    action_ptr_->Execute(req);
  }
  auto start = std::chrono::system_clock::now();
  if (FeedbackTimeout()) {
    response->code = code_ptr_->GetCode(MotionCode::kComLcmTimeout);
    response->result = false;
    response->motion_id = motion_status_ptr_->motion_id;
    ERROR("LCM Com timeout");
    return;
  }
  auto past = std::chrono::system_clock::now() - start;
  std::this_thread::sleep_for(std::chrono::nanoseconds(15 * 1000 * 1000) - past);
  INFO("Wait 15ms over");
  std::unique_lock<std::mutex> check_lk(execute_mutex_);
  wait_id_ = request->motion_id;
  switch (motion_status_ptr_->switch_status) {
    case MotionStatusMsg::BAN_TRANS:
      response->code = code_ptr_->GetCode(MotionCode::kMotionSwitchBantrans);
      response->result = false;
      response->motion_id = motion_status_ptr_->motion_id;
      ERROR("Motion switch BAN_TRANS");
      return;

    case MotionStatusMsg::EDAMP:
      response->code = code_ptr_->GetCode(MotionCode::kMotionSwitchEdamp);
      response->result = false;
      response->motion_id = motion_status_ptr_->motion_id;
      ERROR("Motion switch EDAMP");
      return;

    case MotionStatusMsg::ESTOP:
      response->code = code_ptr_->GetCode(MotionCode::kMotionSwitchEstop);
      response->result = false;
      response->motion_id = motion_status_ptr_->motion_id;
      ERROR("Motion switch ESTOP");
      return;

    case MotionStatusMsg::LIFTED:
      response->code = code_ptr_->GetCode(MotionCode::kMotionSwitchLifted);
      response->result = false;
      response->motion_id = motion_status_ptr_->motion_id;
      ERROR("Motion switch LIFTED");
      return;

    case MotionStatusMsg::OVER_HEAT:
      response->code = code_ptr_->GetCode(MotionCode::kMotionSwitchOverHeat);
      response->result = false;
      response->motion_id = motion_status_ptr_->motion_id;
      ERROR("Motion switch OVER_HEAT");
      return;

    case MotionStatusMsg::LOW_BAT:
      response->code = code_ptr_->GetCode(MotionCode::kMotionSwitchLowBat);
      response->result = false;
      response->motion_id = motion_status_ptr_->motion_id;
      ERROR("Motion switch LOW_BAT");
      return;

    case MotionStatusMsg::TRANSITIONING:
      is_transitioning_wait_ = true;
      WARN("Transitioning waiting");
      if (transitioning_cv_.wait_for(check_lk, std::chrono::milliseconds(kTransitioningTimeout)) ==
        std::cv_status::timeout)
      {
        response->code = code_ptr_->GetCode(MotionCode::kMotionTransitionTimeout);
        response->result = false;
        response->motion_id = motion_status_ptr_->motion_id;
        WARN("Transitioning Timeout");
        is_transitioning_wait_ = false;
        return;
      }
      break;

    default:
      break;
  }
  // if (motion_status_ptr_->switch_status == MotionStatusMsg::BAN_TRANS ||
  //   motion_status_ptr_->switch_status == MotionStatusMsg::EDAMP ||
  //   motion_status_ptr_->switch_status == MotionStatusMsg::ESTOP ||
  //   motion_status_ptr_->switch_status == MotionStatusMsg::LIFTED)
  // {
  //   response->code = code_ptr_->GetCode(MotionCode::kMotionSwitchError);
  //   response->result = false;
  //   response->motion_id = motion_status_ptr_->motion_id;
  //   ERROR("Motion switch error");
  //   return;
  // }
  // if (motion_status_ptr_->switch_status == MotionStatusMsg::TRANSITIONING) {
  //   is_transitioning_wait_ = true;
  //   WARN("Transitioning waiting");
  //   if (transitioning_cv_.wait_for(check_lk, std::chrono::milliseconds(kTransitioningTimeout)) ==
  //     std::cv_status::timeout)
  //   {
  //     response->code = code_ptr_->GetCode(MotionCode::kMotionTransitionTimeout);
  //     response->result = false;
  //     response->motion_id = motion_status_ptr_->motion_id;
  //     WARN("Transitioning Timeout");
  //     is_transitioning_wait_ = false;
  //     return;
  //   }
  // }
  // INFO("Transitioning finished");
  // if (is_transitioning_wait_) {
  //   INFO("Try to relock execute_mutex_");
  //   // check_lk.lock();
  //   INFO("Relock execute_mutex_");
  // }
  is_execute_wait_ = true;

  auto wait_timeout = 0;
  auto min_exec_time = motion_id_map_[request->motion_id].min_exec_time;
  // 站立、趴下、作揖、空翻、绝对力控这些动作运控内部设定了固定时间，duration必须为0
  if (min_exec_time > 0) {
    wait_timeout = min_exec_time;
  } else if (min_exec_time < 0) {  // 增量力控、增量位控、绝对位控、行走duration必须大于0
    wait_timeout = request->duration;
  } else {                         // 自定义动作按照设定的参数计算
    wait_timeout = request->duration == 0 ? 500 : request->duration * 2;
  }
  // INFO("%d", wait_timeout);
  if (execute_cv_.wait_for(
      check_lk,
      std::chrono::milliseconds(wait_timeout)) == std::cv_status::timeout)
  {
    response->code = code_ptr_->GetCode(MotionCode::kMotionExecuteTimeout);
    response->result = false;
    response->motion_id = motion_status_ptr_->motion_id;
    ERROR("Motion execute timeout");
    return;
  }
  int32_t code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  if (!CheckMotionResult(request->motion_id, code)) {
    response->code = code;
    response->result = false;
    response->motion_id = motion_status_ptr_->motion_id;
    ERROR("Motion execute error");
    return;
  }
  response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  response->result = true;
  response->motion_id = motion_status_ptr_->motion_id;
  INFO("Motion %d done", request->motion_id);
}

template
void MotionHandler::HandleResultCmd<MotionResultSrv::Request::SharedPtr,
  MotionResultSrv::Response::SharedPtr>(MotionResultSrv::Request::SharedPtr,
  MotionResultSrv::Response::SharedPtr);

template
void MotionHandler::HandleResultCmd<MotionSequenceShowSrv::Request::SharedPtr,
  MotionSequenceShowSrv::Response::SharedPtr>(MotionSequenceShowSrv::Request::SharedPtr,
  MotionSequenceShowSrv::Response::SharedPtr);

template<typename CmdRequestT, typename CmdResponseT>
void MotionHandler::HandleResultCmd(const CmdRequestT request, CmdResponseT response)
{
  bool DanceInteruption = (GetWorkStatus() == HandlerStatus::kExecutingResultCmd &&
    GetMotionStatus()->motion_id == 140 && request->motion_id == MotionIDMsg::GETDOWN);
  if (DanceInteruption && is_execute_wait_) {
    execute_cv_.notify_all();
  }
  if (GetWorkStatus() != HandlerStatus::kIdle &&
    !DanceInteruption &&
    request->motion_id != MotionIDMsg::ESTOP &&
    request->cmd_source != MotionResultSrv::Request::FSM)
  {
    response->result = false;
    // response->code = code_ptr_->GetCode(MotionCode::kBusy);
    response->code = code_ptr_->GetKeyCode(system::KeyCode::kTargetBusy);
    ERROR("Busy when Getting ResultCmd(%d)", request->motion_id);
    return;
  }
  SetWorkStatus(HandlerStatus::kExecutingResultCmd);
  int32_t code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
  if (!IsCommandValid(request, code)) {
    // response->code = code_ptr_->GetCode(MotionCode::kCommandInvalid);
    response->code = code;
    response->result = false;
    response->motion_id = motion_status_ptr_->motion_id;
    SetWorkStatus(HandlerStatus::kIdle);
    return;
  }
  if (request->motion_id != MotionIDMsg::ESTOP) {
    int32_t code = 0;
    if (!CheckMotors(code)) {
      response->result = false;
      response->code = code;
      ERROR("Motor error");
      SetWorkStatus(HandlerStatus::kIdle);
      return;
    }
    // for (auto motor : motion_status_ptr_->motor_error) {
    //   if (motor != 0 && motor != kMotorNormal) {
    //     response->result = false;
    //     response->code = code_ptr_->GetCode(MotionCode::kHwMotorOffline);
    //     ERROR("Motor error");
    //     SetWorkStatus(HandlerStatus::kIdle);
    //     return;
    //   }
    // }
  }
  CreateTomlLog(request->motion_id);
  if (!CheckPostMotion(request->motion_id)) {
    MotionResultSrv::Request::SharedPtr req(new MotionResultSrv::Request);
    MotionResultSrv::Response::SharedPtr res(new MotionResultSrv::Response);
    req->motion_id = MotionIDMsg::RECOVERYSTAND;
    INFO("Trying to be ready for ResultCmd");
    ExecuteResultCmd(req, res);
    if (!res->result) {
      response->code = res->code;
      response->result = false;
      response->motion_id = motion_status_ptr_->motion_id;
      ERROR("Get error when trying to be ready for ResultCmd");
      CloseTomlLog();
      SetWorkStatus(HandlerStatus::kIdle);
      return;
    }
  }
  if (request->motion_id == MotionIDMsg::SEQUENCE_CUSTOM) {
    INFO("\n%s", request->gait_toml.c_str());
    if (!action_ptr_->SequenceDefImpl(request->gait_toml)) {
      response->code = code_ptr_->GetCode(MotionCode::kSequenceDefError);
      response->result = false;
      response->motion_id = motion_status_ptr_->motion_id;
      ERROR("SequenceCmd(%d) defination error", request->motion_id);
      SetWorkStatus(HandlerStatus::kIdle);
      CloseTomlLog();
      return;
    }
  }
  if (request->motion_id == MotionIDMsg::TWO_LEG_STAND) {
    action_ptr_->ShowBlackSkin();
  }
  // if (request->motion_id == MotionIDMsg::RECOVERYSTAND) {
  //   if(elec_skin_id_ >= 3) {
  //     elec_skin_id_ = 0;
  //   }
  //   switch (elec_skin_id_)
  //   {
  //     case 0:
  //       action_ptr_->ShowStandElecSkin();
  //       break;

  //     case 1:
  //       action_ptr_->ShowTwinkElecSkin();
  //       break;

  //     case 2:
  //       action_ptr_->ShowRandomElecSkin();
  //       break;

  //     default:
  //       break;
  //   }
  //   elec_skin_id_++;
  // }
  // if (request->motion_id == MotionIDMsg::GETDOWN ||
  //   request->motion_id == 143) { // 坐下
  //   elec_skin_id_ = 0;
  //   action_ptr_->ShowStandElecSkin();
  // }
  if (request->motion_id == 140) {
    INFO("Will sing");
    Sing(dance_map_[140]);
    // action_ptr_->SetAlignContact(true);
  }
  if (request->motion_id == 176) {
    INFO("Will sing");
    Sing(dance_map_[176]);
  }
  if (request->motion_id == 177) {
    INFO("Will sing");
    Sing(dance_map_[177]);
  }
  if (request->motion_id == 178) {
    INFO("Will sing");
    Sing(dance_map_[178]);
  }
  if (request->motion_id == MotionIDMsg::GETDOWN) {
    // action_ptr_->ShowDefaultSkin(true, true);
    INFO("Stop sing");
    Sing(dance_map_[101]);
    // sing_ = false;
  }
  // if (request->motion_id == MotionIDMsg::BACK_FLIP) {
  //   action_ptr_->ShowDefaultSkin(true, true);
  // }
  ExecuteResultCmd(request, response);
  if (request->motion_id == MotionIDMsg::TWO_LEG_STAND) {
    action_ptr_->ShowWhiteSkin();
  }
  CloseTomlLog();
  SetWorkStatus(HandlerStatus::kIdle);
  INFO("Will return HandleResultCmd");
}

// void MotionHandler::HandleSequenceCmd(
//   const MotionSequenceSrv::Request::SharedPtr request,
//   MotionSequenceSrv::Response::SharedPtr response)
// {
//   if (GetWorkStatus() != HandlerStatus::kIdle) {
//     response->result = false;
//     // response->code = code_ptr_->GetCode(MotionCode::kBusy);
//     response->code = code_ptr_->GetKeyCode(system::KeyCode::kTargetBusy);
//     ERROR("Busy when Getting SequenceCmd(%d)", MotionIDMsg::SEQUENCE_CUSTOM);
//     return;
//   }
//   SetWorkStatus(HandlerStatus::kExecutingResultCmd);
//   auto req = std::make_shared<MotionResultSrv::Request>();
//   req->motion_id = MotionIDMsg::SEQUENCE_CUSTOM;
//   int32_t code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
//   if (!IsCommandValid(req, code)) {
//     // response->code = code_ptr_->GetCode(MotionCode::kCommandInvalid);
//     response->code = code;
//     response->result = false;
//     response->describe = "";
//     SetWorkStatus(HandlerStatus::kIdle);
//     return;
//   }
//   CreateTomlLog(req->motion_id);
//   ExecuteResultCmd(request, response);
//   CloseTomlLog();
//   SetWorkStatus(HandlerStatus::kIdle);
// }


void MotionHandler::HandleQueueCmd(
  const MotionQueueCustomSrv::Request::SharedPtr request,
  MotionQueueCustomSrv::Response::SharedPtr response)
{
  // if (GetWorkStatus() != HandlerStatus::kIdle && request->motion_id != MotionIDMsg::ESTOP) {
  //   response->result = false;
  //   response->code = MotionCodeMsg::TASK_STATE_ERROR;
  //   return;
  // }
  // SetWorkStatus(HandlerStatus::kExecutingResultCmd);
  // if (!IsCommandValid(request)) {
  //   response->code = MotionCodeMsg::COMMAND_INVALID;
  //   response->result = false;
  //   response->motion_id = motion_status_ptr_->motion_id;
  //   SetWorkStatus(HandlerStatus::kIdle);
  //   return;
  // }
  (void)response;
  CreateTomlLog("queue");
  action_ptr_->Execute(request);
  CloseTomlLog();
  SetWorkStatus(HandlerStatus::kIdle);
}

void MotionHandler::UpdateMotionStatus(const MotionStatusMsg::SharedPtr & motion_status_ptr)
{
  static int8_t count = 5;
  if (count > 0) {
    INFO("Get Feedback: %d", count);
    --count;
  }
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
  feedback_cv_.notify_one();
  if (is_transitioning_wait_ &&
    motion_status_ptr_->motion_id == wait_id_ &&
    motion_status_ptr_->switch_status == MotionStatusMsg::NORMAL)
  {
    transitioning_cv_.notify_one();
    is_transitioning_wait_ = false;
  }
  if (is_execute_wait_ &&
    motion_status_ptr_->motion_id == wait_id_ &&
    motion_status_ptr_->order_process_bar > 95)
  {
    execute_cv_.notify_one();
    is_execute_wait_ = false;
  }
}

MotionStatusMsg::SharedPtr MotionHandler::GetMotionStatus()
{
  return motion_status_ptr_;
}

bool MotionHandler::CheckPostMotion(int32_t motion_id)
{
  if (motion_id == MotionIDMsg::RECOVERYSTAND || motion_id == MotionIDMsg::ESTOP) {
    return true;
  }
  if (motion_status_ptr_->motion_id == -1) {
    return false;
  }

  // 请求的mode
  int32_t request_mode = motion_id_map_.find(motion_id)->second.map.front();
  // 当前状态允许切换的post_motion
  std::vector<int32_t> post_motion =
    motion_id_map_.find(motion_status_ptr_->motion_id)->second.post_motion;

  return std::find(post_motion.begin(), post_motion.end(), request_mode) != post_motion.end();
}

bool MotionHandler::AllowServoCmd(int32_t motion_id)
{
  // TODO(harvey): 判断当前状态是否能够行走
  if (post_motion_checked_) {return true;}
  return CheckPostMotion(motion_id);
}

void MotionHandler::WriteTomlLog(const robot_control_cmd_lcmt & cmd)
{
  if (!toml_.is_open()) {
    // ERROR("TomlLog File not set before writing");
    return;
  }
  toml_ << "# " + GetCurrentTime() << "\n";
  toml_ << "[[step]]\n";
  toml_ << "mode = " << int(cmd.mode) << "\n";
  toml_ << "gait_id = " << int(cmd.gait_id) << "\n";
  toml_ << "contact = " << int(cmd.contact) << "\n";
  toml_ << "life_count = " << int(cmd.life_count) << "\n";
  toml_ << "value = " << cmd.value << "\n";
  toml_ << "duration = " << cmd.duration << "\n";
  toml_ << "vel_des = [ " <<
    cmd.vel_des[0] << ", " <<
    cmd.vel_des[1] << ", " <<
    cmd.vel_des[2] << ",]\n";
  toml_ << "rpy_des = [ " <<
    cmd.rpy_des[0] << ", " <<
    cmd.rpy_des[1] << ", " <<
    cmd.rpy_des[2] << ",]\n";
  toml_ << "pos_des = [ " <<
    cmd.pos_des[0] << ", " <<
    cmd.pos_des[1] << ", " <<
    cmd.pos_des[2] << ",]\n";
  toml_ << "acc_des = [ " <<
    cmd.acc_des[0] << ", " <<
    cmd.acc_des[1] << ", " <<
    cmd.acc_des[2] << ", " <<
    cmd.acc_des[3] << ", " <<
    cmd.acc_des[4] << ", " <<
    cmd.acc_des[5] << ",]\n";
  toml_ << "ctrl_point = [ " <<
    cmd.ctrl_point[0] << ", " <<
    cmd.ctrl_point[1] << ", " <<
    cmd.ctrl_point[2] << ",]\n";
  toml_ << "foot_pose = [ " <<
    cmd.foot_pose[0] << ", " <<
    cmd.foot_pose[1] << ", " <<
    cmd.foot_pose[2] << ", " <<
    cmd.foot_pose[3] << ", " <<
    cmd.foot_pose[4] << ", " <<
    cmd.foot_pose[5] << ",]\n";
  toml_ << "step_height = [ " <<
    cmd.step_height[0] << ", " <<
    cmd.step_height[1] << ",]\n";
  toml_ << "\n";
}

bool MotionHandler::CheckMotors(int32_t & code)
{
  bool ret = true;
  bool fatal_error = false;
  bool over_heat_error = false;
  int motor = 0;
  auto fatal = std::set<uint8_t>{0, 2, 8};
  auto over_heat = std::set<uint8_t>{1, 24};
  for (auto motor_error : motion_status_ptr_->motor_error) {
    int pos = 0;
    while (pos <= 29) {
      if (motor_error & (1 << pos)) {
        ERROR("Motor %d : %d", motor, pos);
        if (fatal.find(pos) != fatal.end()) {
          fatal_error = true;
        }
        if (over_heat.find(pos) != over_heat.end()) {
          over_heat_error = true;
        }
        ret = false;
      }
      ++pos;
    }
    ++motor;
  }
  if (ret) {
    return true;
  }
  if (fatal_error) {
    code = code_ptr_->GetCode(MotionCode::kHwMotorOffline);
  } else if (over_heat_error) {
    code = code_ptr_->GetCode(MotionCode::kHwMotorOverHeat);
  } else {
    code = code_ptr_->GetCode(MotionCode::kHwMotorOverLoad);
  }
  return false;
}

void MotionHandler::SetDanceMap()
{
  dance_map_[101] = 9999;
  dance_map_[140] = 60003;
  dance_map_[176] = 60005;
  dance_map_[177] = 60006;
  dance_map_[178] = 60007;
}
}  // namespace motion
}  // namespace cyberdog
