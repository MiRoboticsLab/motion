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
: machine::MachineActuator(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
}

MotionManager::~MotionManager()
{}

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

  servo_response_pub_ = node_ptr_->create_publisher<MotionServoResponseMsg>(
    kMotionServoResponseTopicName, 10);

  decision_ptr_ = std::make_shared<MotionDecision>(node_ptr_, code_ptr_);
  decision_ptr_->Init(servo_response_pub_);
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
  motion_sequence_show_srv_ =
    node_ptr_->create_service<MotionSequenceShowSrv>(
    kMotionSequenceServiceName,
    std::bind(
      &MotionManager::MotionSequenceShowCmdCallback, this, std::placeholders::_1,
      std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  audio_client_ = node_ptr_->create_client<protocol::srv::AudioTextPlay>(
    "speech_text_play",
    rmw_qos_profile_services_default,
    callback_group_);
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/manager/state_machine_config.toml");
  heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeatsActuator>("motion_manager");
  heart_beats_ptr_->HeartBeatRun();
  if (!this->MachineActuatorInit(path, node_ptr_)) {
    ERROR("Init failed, actuator init error.");
    return false;
  }
  this->RegisterStateCallback("SetUp", std::bind(&MotionManager::OnSetUp, this));
  this->RegisterStateCallback("TearDown", std::bind(&MotionManager::OnTearDown, this));
  this->RegisterStateCallback("SelfCheck", std::bind(&MotionManager::OnSelfCheck, this));
  this->RegisterStateCallback("Active", std::bind(&MotionManager::OnActive, this));
  this->RegisterStateCallback("DeActive", std::bind(&MotionManager::OnDeActive, this));
  this->RegisterStateCallback("Protected", std::bind(&MotionManager::OnProtected, this));
  this->RegisterStateCallback("LowPower", std::bind(&MotionManager::OnLowPower, this));
  this->RegisterStateCallback("OTA", std::bind(&MotionManager::OnOTA, this));
  this->RegisterStateCallback("Error", std::bind(&MotionManager::OnError, this));
  if (!this->ActuatorStart()) {
    ERROR("Init failed, actuator start error.");
    return false;
  }
  status_map_.emplace(MotionMgrState::kUninit, "Uninit");
  status_map_.emplace(MotionMgrState::kSetup, "Setup");
  status_map_.emplace(MotionMgrState::kTearDown, "TearDown");
  status_map_.emplace(MotionMgrState::kSelfCheck, "SelfCheck");
  status_map_.emplace(MotionMgrState::kActive, "Active");
  status_map_.emplace(MotionMgrState::kDeactive, "Deactive");
  status_map_.emplace(MotionMgrState::kProtected, "Protected");
  status_map_.emplace(MotionMgrState::kLowPower, "LowPower");
  status_map_.emplace(MotionMgrState::kOTA, "OTA");
  status_map_.emplace(MotionMgrState::kError, "Error");
  thread_ = std::make_unique<std::thread>(&MotionManager::Report, this);
  thread_->detach();
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

bool MotionManager::IsStateValid(int32_t & code, bool protected_cmd)
{
  auto state = GetState();
  if (state == MotionMgrState::kActive ||
    (state == MotionMgrState::kProtected && !protected_cmd))
  {
    code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
    return true;
  }
  if (state == MotionMgrState::kProtected) {
    code = code_ptr_->GetKeyCode(system::KeyCode::kProtectedError);
    return false;
  }
  code = code_ptr_->GetKeyCode(system::KeyCode::kStateInvalid);
  return false;
}

int32_t MotionManager::OnSetUp()
{
  INFO("Get fsm: Setup");
  SetState(MotionMgrState::kSetup);
  return code_ptr_->GetKeyCode(system::KeyCode::kOK);
}

int32_t MotionManager::OnTearDown()
{
  INFO("Get fsm: TearDown");
  SetState(MotionMgrState::kTearDown);
  if (decision_ptr_->GetMotionID() != MotionIDMsg::ESTOP &&
    decision_ptr_->GetMotionID() != MotionIDMsg::GETDOWN)
  {
    while (!TryGetDownOnFsm(true) && rclcpp::ok()) {
      INFO("Error when GetDown on TearDown, Will retry");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } else {
    INFO("Estop or Getdown, will do nothing when into TearDown");
  }
  return code_ptr_->GetKeyCode(system::KeyCode::kOK);
}

int32_t MotionManager::OnSelfCheck()
{
  INFO("Get fsm: SelfCheck");
  SetState(MotionMgrState::kSelfCheck);
  if (decision_ptr_->SelfCheck()) {
    INFO("MotionManager SelfCheck OK");
    return code_ptr_->GetKeyCode(system::KeyCode::kOK);
  } else {
    ERROR("MotionManager SelfCheck Error");
    return code_ptr_->GetKeyCode(system::KeyCode::kSelfCheckFailed);
  }
}

int32_t MotionManager::OnActive()
{
  INFO("Get fsm: Active");
  SetState(MotionMgrState::kActive);
  return code_ptr_->GetKeyCode(system::KeyCode::kOK);
}

int32_t MotionManager::OnDeActive()
{
  INFO("Get fsm: Deactive");
  SetState(MotionMgrState::kDeactive);
  return code_ptr_->GetKeyCode(system::KeyCode::kOK);
}

int32_t MotionManager::OnProtected()
{
  INFO("Get fsm: Protected");
  SetState(MotionMgrState::kProtected);
  return code_ptr_->GetKeyCode(system::KeyCode::kOK);
}

int32_t MotionManager::OnLowPower()
{
  INFO("Get fsm: LowPower");
  SetState(MotionMgrState::kLowPower);
  if (decision_ptr_->GetMotionID() != MotionIDMsg::ESTOP &&
    decision_ptr_->GetMotionID() != MotionIDMsg::GETDOWN)
  {
    while (!TryGetDownOnFsm() && rclcpp::ok()) {
      INFO("Error when GetDown on LowPower, Will retry");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } else {
    INFO("Estop or Getdown, will do nothing when into LowPower");
  }
  return code_ptr_->GetKeyCode(system::KeyCode::kOK);
}

int32_t MotionManager::OnOTA()
{
  INFO("Get fsm: OTA");
  SetState(MotionMgrState::kOTA);
  if (decision_ptr_->GetMotionID() != MotionIDMsg::ESTOP &&
    decision_ptr_->GetMotionID() != MotionIDMsg::GETDOWN)
  {
    while (!TryGetDownOnFsm() && rclcpp::ok()) {
      INFO("Error when GetDown on OTA, Will retry");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } else {
    INFO("Estop or Getdown, will do nothing when into OTA");
  }
  return code_ptr_->GetKeyCode(system::KeyCode::kOK);
}

int32_t MotionManager::OnError()
{
  INFO("Get fsm: OTA");
  SetState(MotionMgrState::kError);
  return code_ptr_->GetKeyCode(system::KeyCode::kOK);
}

void MotionManager::MotionServoCmdCallback(const MotionServoCmdMsg::SharedPtr msg)
{
  INFO("Receive ServoCmd from %d with motion_id: %d", msg->cmd_source, msg->motion_id);
  int32_t code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
  if (!IsStateValid(code, false)) {
    ERROR("FSM invalid with current state: %s", status_map_.at(state_).c_str());
    static auto servo_response_msg = std::make_shared<MotionServoResponseMsg>();
    servo_response_msg->result = false;
    servo_response_msg->code = code;
    servo_response_pub_->publish(*servo_response_msg);
    return;
  }
  decision_ptr_->DecideServoCmd(msg);
}

void MotionManager::MotionResultCmdCallback(
  const MotionResultSrv::Request::SharedPtr request, MotionResultSrv::Response::SharedPtr response)
{
  INFO("Receive ResultCmd from %d with motion_id: %d", request->cmd_source, request->motion_id);
  int32_t code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
  bool protected_cmd =
    (request->motion_id != MotionIDMsg::RECOVERYSTAND &&
    request->motion_id != MotionIDMsg::GETDOWN &&
    request->motion_id != 102 &&
    request->motion_id != MotionIDMsg::ESTOP &&
    request->motion_id != MotionIDMsg::POSECONTROL_RELATIVEYLY);
  if (!IsStateValid(code, protected_cmd)) {
    ERROR("FSM invalid with current state: %s", status_map_.at(state_).c_str());
    if (code == code_ptr_->GetKeyCode(system::KeyCode::kProtectedError)) {
      OnlineAudioPlay("电量低，请充电后尝试");
    }
    response->result = false;
    response->code = code;
    return;
  }

  decision_ptr_->DecideResultCmd(request, response);
  if (decision_ptr_->IsErrorCode(response->code)) {
    std::unique_lock<std::mutex> lock(msg_mutex_);
    code_ = response->code;
    motion_id_ = response->motion_id;
    msg_condition_.notify_one();
  }
  INFO("Will return MotionResultCmdCallback");
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


void MotionManager::MotionSequenceShowCmdCallback(
  const MotionSequenceShowSrv::Request::SharedPtr request,
  MotionSequenceShowSrv::Response::SharedPtr response)
{
  INFO("Receive SequenceShowCmd with motion_id: %d", request->motion_id);
  int32_t code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
  if (!IsStateValid(code)) {
    ERROR("FSM invalid with current state: %s", status_map_.at(state_).c_str());
    response->result = false;
    response->code = code;
    return;
  }
  // int64_t total_duration = 0;
  // for (auto & pace : request->pace_list) {
  //   total_duration += pace.duration;
  // }
  // decision_ptr_->SetSequnceTotalDuration(total_duration);
  decision_ptr_->DecideResultCmd(request, response);
  if (decision_ptr_->IsErrorCode(response->code)) {
    std::unique_lock<std::mutex> lock(msg_mutex_);
    code_ = response->code;
    motion_id_ = response->motion_id;
    msg_condition_.notify_one();
  }
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
  int32_t code = code_ptr_->GetKeyCode(system::KeyCode::kOK);
  if (!IsStateValid(code)) {
    INFO("State invalid with current state");
    return;
  }
  decision_ptr_->DecideQueueCmd(request, response);
}

void MotionManager::Report()
{
  while (true) {
    std::unique_lock<std::mutex> lock(msg_mutex_);
    msg_condition_.wait(lock);
    decision_ptr_->ReportErrorCode(code_, motion_id_);

    std::this_thread::sleep_for(std::chrono::microseconds(20));
  }
}

}  // namespace motion
}  // namespace cyberdog
