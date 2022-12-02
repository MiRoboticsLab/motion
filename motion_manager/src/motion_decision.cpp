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
#include <vector>
#include <string>
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_manager/motion_decision.hpp"

namespace cyberdog
{
namespace motion
{

MotionDecision::MotionDecision(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MCode> & code)
: node_ptr_(node), code_ptr_(code)
{}

MotionDecision::~MotionDecision() {}

void MotionDecision::Config() {}

bool MotionDecision::Init(rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub)
{
  handler_ptr_ = std::make_shared<MotionHandler>(node_ptr_, code_ptr_);
  if (!handler_ptr_->Init()) {
    ERROR("Fail to initialize MotionHandler");
    return false;
  }
  handler_ptr_->RegisterModeFunction(std::bind(&MotionDecision::ResetMode, this));
  std::string toml_file = ament_index_cpp::get_package_share_directory(
    "motion_manager") + "/config/priority.toml";
  toml::value config;
  if (!cyberdog::common::CyberdogToml::ParseFile(toml_file, config)) {
    FATAL("Cannot parse %s", toml_file.c_str());
    exit(-1);
  }
  GET_TOML_VALUE(config, "App", priority_map_[int32_t(DecisionStatus::kExecutingApp)]);
  GET_TOML_VALUE(config, "Audio", priority_map_[int32_t(DecisionStatus::kExecutingAudio)]);
  GET_TOML_VALUE(config, "Vis", priority_map_[int32_t(DecisionStatus::kExecutingVis)]);
  GET_TOML_VALUE(config, "BluTele", priority_map_[int32_t(DecisionStatus::kExecutingBluTele)]);
  GET_TOML_VALUE(config, "Algo", priority_map_[int32_t(DecisionStatus::kExecutingAlgo)]);
  priority_map_[int32_t(DecisionStatus::kIdle)] = int32_t(DecisionStatus::kIdle);
  priority_map_[int32_t(DecisionStatus::kExecutingKeyBoardDebug)] =
    int32_t(DecisionStatus::kExecutingKeyBoardDebug);
  servo_response_pub_ = servo_response_pub;
  servo_response_thread_ = std::thread(std::bind(&MotionDecision::ServoResponseThread, this));
  servo_response_thread_.detach();
  laser_helper_ = std::make_shared<LaserHelper>(node_ptr_);
  ResetServoResponseMsg();
  return true;
}

void MotionDecision::DecideServoCmd(const MotionServoCmdMsg::SharedPtr & msg)
{
  SetServoResponse();
  if (!IsStateValid(msg->motion_id)) {
    servo_response_msg_.motion_id = handler_ptr_->GetMotionStatus()->motion_id;
    servo_response_msg_.result = false;
    servo_response_msg_.code = code_ptr_->GetCode(MotionCode::kEstop);
    ERROR("Forbidden ServoCmd when estop");
    return;
  }
  if (!IsModeValid(msg->cmd_source)) {
    servo_response_msg_.code = code_ptr_->GetKeyCode(system::KeyCode::kFailed);
    ERROR("Mode error, %d in control when get %d", (int32_t)motion_work_mode_, msg->cmd_source);
    return;
  }
  SetMode((DecisionStatus)msg->cmd_source);
  handler_ptr_->HandleServoCmd(msg, servo_response_msg_);
}

/**
 * @brief 伺服指令反馈线程，运行在死循环线程中
 *        通过NeedServerResponse来挂起 / 唤醒线程
 *
 */
void MotionDecision::ServoResponseThread()
{
  while (true) {
    if (!NeedServoResponse()) {
      continue;
    }
    if (servo_response_pub_ != nullptr) {
      // FIXME(harvey): 伺服指令的运行结果判断机制
      if (!handler_ptr_->FeedbackTimeout()) {
        auto motion_status = handler_ptr_->GetMotionStatus();
        servo_response_msg_.motion_id = motion_status->motion_id;
        servo_response_msg_.order_process_bar = motion_status->order_process_bar;
        servo_response_msg_.status = motion_status->switch_status;
        int32_t code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
        handler_ptr_->CheckMotionResult(code);
        servo_response_msg_.code = code;
        servo_response_pub_->publish(servo_response_msg_);
      } else {
        servo_response_msg_.motion_id = -1;
        servo_response_msg_.order_process_bar = -1;
        servo_response_msg_.status = -1;
        servo_response_msg_.result = false;
        servo_response_msg_.code = code_ptr_->GetCode(MotionCode::kComLcmTimeout);
        servo_response_pub_->publish(servo_response_msg_);
      }
    }
  }
}

template
void MotionDecision::DecideResultCmd<MotionResultSrv::Request::SharedPtr,
  MotionResultSrv::Response::SharedPtr>(MotionResultSrv::Request::SharedPtr,
  MotionResultSrv::Response::SharedPtr);

template
void MotionDecision::DecideResultCmd<MotionSequenceSrv::Request::SharedPtr,
  MotionSequenceSrv::Response::SharedPtr>(MotionSequenceSrv::Request::SharedPtr,
  MotionSequenceSrv::Response::SharedPtr);

/**
 * @brief 执行结果指令
 *
 * @param request
 * @param response
 */
template<typename CmdRequestT, typename CmdResponseT>
void MotionDecision::DecideResultCmd(
  const CmdRequestT request, CmdResponseT response)
{
  int32_t error_code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  if (!IsStateValid(request->motion_id, error_code)) {
    response->motion_id = handler_ptr_->GetMotionStatus()->motion_id;
    response->result = false;
    response->code = error_code;
    return;
  }
  if (!IsModeValid()) {
    return;
  }
  estop_ = request->motion_id == MotionIDMsg::ESTOP;
  handler_ptr_->HandleResultCmd(request, response);
}

// /**
//  * @brief 执行自定义动作指令
//  *
//  * @param request
//  * @param response
//  */
// void MotionDecision::DecideSequenceCmd(
//   const MotionSequenceSrv::Request::SharedPtr request,
//   MotionSequenceSrv::Response::SharedPtr response)
// {
//   int32_t error_code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
//   if (!IsStateValid(MotionIDMsg::SEQUENCE_CUSTOM, error_code)) {
//     response->result = false;
//     response->code = error_code;
//     return;
//   }
//   if (!IsModeValid()) {
//     return;
//   }
//   handler_ptr_->HandleSequenceCmd(request, response);
// }

/**
 * @brief 执行结果指令
 *
 * @param request
 * @param response
 */
void MotionDecision::DecideQueueCmd(
  const MotionQueueCustomSrv::Request::SharedPtr request,
  MotionQueueCustomSrv::Response::SharedPtr response)
{
  if (!IsStateValid()) {
    response->result = false;
    return;
  }
  if (!IsModeValid()) {
    return;
  }
  handler_ptr_->HandleQueueCmd(request, response);
}

}  // namespace motion
}  // namespace cyberdog
