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
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_manager/motion_manager.hpp"

namespace cyberdog
{
namespace motion
{
MotionManager::MotionManager(const std::string & name)
: manager::ManagerBase(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
}

MotionManager::~MotionManager()
{}

void MotionManager::Config()
{
  INFO("Get info from configure");
}

bool MotionManager::Init()
{
  INFO("Init on call");
  if (node_ptr_ == nullptr) {
    ERROR("Init failed with nullptr at ros node!");
    return false;
  }
  code_ptr_ = std::make_shared<MCode>(cyberdog::system::ModuleCode::kMotionManager);
  executor_.reset(new rclcpp::executors::MultiThreadedExecutor);
  callback_group_ = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  decision_ptr_ = std::make_shared<MotionDecision>(node_ptr_, code_ptr_);
  decision_ptr_->Init();

  motion_servo_sub_ = node_ptr_->create_subscription<MotionServoCmdMsg>(
    kMotionServoCommandTopicName, rclcpp::SystemDefaultsQoS(),
    std::bind(&MotionManager::MotionServoCmdCallback, this, std::placeholders::_1));
  motion_result_srv_ =
    node_ptr_->create_service<MotionResultSrv>(
    kMotionResultServiceName,
    std::bind(
      &MotionManager::MotionResultCmdCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  motion_custom_srv_ =
    node_ptr_->create_service<MotionCustomSrv>(
    kMotionCustomServiceName,
    std::bind(
      &MotionManager::MotionCustomCmdCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  motion_queue_srv_ =
    node_ptr_->create_service<MotionQueueCustomSrv>(
    kMotionQueueServiceName,
    std::bind(
      &MotionManager::MotionQueueCmdCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  motion_sequence_srv_ =
    node_ptr_->create_service<MotionSequenceSrv>(
    kMotionSequenceServiceName,
    std::bind(
      &MotionManager::MotionSequenceCmdCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  return true;
}

void MotionManager::Run()
{
  INFO("Running on...");
  // rclcpp::spin(node_ptr_);
  executor_->add_node(node_ptr_);
  executor_->spin();
  rclcpp::shutdown();
}

bool MotionManager::SelfCheck()
{
  // check all motions from config
  return true;
}

bool MotionManager::IsStateValid()
{
  // check state from behavior tree
  return true;
}

void MotionManager::OnError()
{
  INFO("on error");
}

void MotionManager::OnLowPower()
{
  INFO("on lowpower");
}

void MotionManager::OnSuspend()
{
  INFO("on suspend");
}

void MotionManager::OnProtected()
{
  INFO("on protect");
}

void MotionManager::OnActive()
{
  INFO("on active");
}

void MotionManager::MotionServoCmdCallback(const MotionServoCmdMsg::SharedPtr msg)
{
  INFO("Receive ServoCmd from %d with motion_id: %d", msg->cmd_source, msg->motion_id);
  if (!IsStateValid()) {
    INFO("motion state invalid with current state");
    return;
  }

  decision_ptr_->DecideServoCmd(msg);
}

void MotionManager::MotionResultCmdCallback(
  const MotionResultSrv::Request::SharedPtr request, MotionResultSrv::Response::SharedPtr response)
{
  INFO("Receive ResultCmd from %d with motion_id: %d", request->cmd_source, request->motion_id);
  if (!IsStateValid()) {
    INFO("State invalid with current state");
    return;
  }

  decision_ptr_->DecideResultCmd(request, response);
}

void MotionManager::MotionCustomCmdCallback(
  const MotionCustomSrv::Request::SharedPtr, MotionCustomSrv::Response::SharedPtr)
{
  // if (request->cmd_type == MotionCustomSrv::Request::DEFINITION) {
  //   auto lcm = std::make_shared<lcm::LCM>(kLCMBirdgeSubscribeURL);
  //   std::ifstream custom_config(kMotionCustomCmdConfigPath);
  //   std::string s;
  //   file_send_lcmt lcm_file;
  //   while (getline(custom_config, s)) {
  //     INFO("%s", s.c_str());
  //     lcm_file.data += s + "\n";
  //   }
  //   if (0 == lcm->publish(kLCMBridgeFileChannel, &lcm_file)) {
  //     response->result = true;
  //     response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  //   } else {
  //     response->result = false;
  //     response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  //   }
  //   // TODO (Harvey): motion_id如何返回
  // } else if (request->cmd_type == MotionCustomSrv::Request::EXECUTION) {
  //   INFO("Receive Cmd with motion_id: %d", request->motion_id);
  //   if (!IsStateValid()) {
  //     INFO("State invalid with current state");
  //     return;
  //   }
  //   auto req = std::make_shared<MotionResultSrv::Request>();
  //   req->motion_id = request->motion_id;
  //   req->cmd_source = request->cmd_source;
  //   auto res = std::make_shared<MotionResultSrv::Response>();
  //   decision_ptr_->DecideResultCmd(req, res);
  //   response->motion_id = res->motion_id;
  //   response->result = res->result;
  //   response->code = res->code;
  //   // decision_ptr_->DecideCustomCmd(request, response);
  // }
}


void MotionManager::MotionSequenceCmdCallback(
  const MotionSequenceSrv::Request::SharedPtr request,
  MotionSequenceSrv::Response::SharedPtr response)
{
  INFO("Receive SequenceCmd with motion_id: %d", request->motion_id);
  if (!IsStateValid()) {
    INFO("State invalid with current state");
    return;
  }
  int64_t total_duration = 0;
  for (auto & param : request->params) {
    total_duration += param.duration_ms;
  }
  decision_ptr_->SetSequnceTotalDuration(total_duration);
  decision_ptr_->DecideResultCmd(request, response);

  // if (request->cmd_type == MotionCustomSrv::Request::DEFINITION) {
  //   auto lcm = std::make_shared<lcm::LCM>(kLCMBirdgeSubscribeURL);
  //   std::ifstream custom_config(kMotionCustomCmdConfigPath);
  //   std::string s;
  //   file_lcmt lcm_file;
  //   while (getline(custom_config, s)) {
  //     INFO("%s", s.c_str());
  //     lcm_file.data += s + "\n";
  //   }
  //   if (0 == lcm->publish(kLCMBridgeFileChannel, &lcm_file)) {
  //     response->result = true;
  //     response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  //   } else {
  //     response->result = false;
  //     response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  //   }
  //   // TODO (Harvey): motion_id如何返回
  // } else if (request->cmd_type == MotionCustomSrv::Request::EXECUTION) {
  //   INFO("Receive Cmd with motion_id: %d", request->motion_id);
  //   if (!IsStateValid()) {
  //     INFO("State invalid with current state");
  //     return;
  //   }
  //   auto req = std::make_shared<MotionResultSrv::Request>();
  //   req->motion_id = request->motion_id;
  //   req->cmd_source = request->cmd_source;
  //   auto res = std::make_shared<MotionResultSrv::Response>();
  //   decision_ptr_->DecideResultCmd(req, res);
  //   response->motion_id = res->motion_id;
  //   response->result = res->result;
  //   response->code = res->code;
  //   // decision_ptr_->DecideCustomCmd(request, response);
  // }
}

void MotionManager::MotionQueueCmdCallback(
  const MotionQueueCustomSrv::Request::SharedPtr request,
  MotionQueueCustomSrv::Response::SharedPtr response)
{
  INFO("Receive QueueCmd");
  if (!IsStateValid()) {
    INFO("State invalid with current state");
    return;
  }
  decision_ptr_->DecideQueueCmd(request, response);
}

}  // namespace motion
}  // namespace cyberdog
