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
#include <map>
#include <vector>
#include <fstream>
#include "elec_skin/elec_skin.hpp"
#include "motion_action/motion_action.hpp"

namespace cyberdog
{
namespace motion
{
MotionAction::MotionAction()
{}

MotionAction::~MotionAction() {}

void MotionAction::Execute(const MotionServoCmdMsg::SharedPtr msg)
{
  // Checkout mode global, send msg continuously
  if (!ins_init_) {
    ERROR("MotionAction has not been initialized when execute ServoCmd");
    return;
  }
  if (motion_id_map_.empty()) {
    return;
  }
  static auto lcm_cmd = std::make_shared<robot_control_cmd_lcmt>();
  lcm_cmd->mode = motion_id_map_.at(msg->motion_id).map.front();
  lcm_cmd->gait_id = motion_id_map_.at(msg->motion_id).map.back();
  lcm_cmd->contact = 0;
  lcm_cmd->value = msg->value;
  lcm_cmd->duration = 0;
  GET_VALUE(msg->step_height, lcm_cmd->step_height, 2, "step_height");
  GET_VALUE(msg->vel_des, lcm_cmd->vel_des, 3, "vel_des");
  GET_VALUE(msg->rpy_des, lcm_cmd->rpy_des, 3, "rpy_des");
  GET_VALUE(msg->pos_des, lcm_cmd->pos_des, 3, "pos_des");
  GET_VALUE(msg->ctrl_point, lcm_cmd->ctrl_point, 3, "ctrl_point");
  GET_VALUE(msg->acc_des, lcm_cmd->acc_des, 6, "acc_des");
  GET_VALUE(msg->foot_pose, lcm_cmd->foot_pose, 6, "foot_pose");
  std::unique_lock<std::mutex> lk(lcm_write_mutex_);
  lcm_cmd_ = *lcm_cmd;
  lcm_cmd_.life_count = life_count_++;
  lcm_publish_instance_->publish(kLCMActionControlChannel, &lcm_cmd_);
  lk.unlock();
  lcm_cmd_init_ = true;
  INFO(
    "ServoCmd: %d, %d, %d, %d", lcm_cmd_.mode, lcm_cmd_.gait_id, lcm_cmd_.life_count,
    lcm_cmd_.duration);
}

void MotionAction::Execute(const robot_control_cmd_lcmt & lcm)
{
  std::unique_lock<std::mutex> lk(lcm_write_mutex_);
  lcm_cmd_ = lcm;
  lcm_cmd_.life_count = life_count_++;
  lcm_publish_instance_->publish(kLCMActionControlChannel, &lcm_cmd_);
  lk.unlock();
  lcm_cmd_init_ = true;
  INFO(
    "ResultCmd: %d, %d, %d, %d", lcm_cmd_.mode, lcm_cmd_.gait_id, lcm_cmd_.life_count,
    lcm_cmd_.duration);
  if (toml_log_func_) {
    toml_log_func_(lcm_cmd_);
  }
}

void MotionAction::Execute(const MotionResultSrv::Request::SharedPtr request)
{
  if (!ins_init_) {
    ERROR("MotionAction has not been initialized when execute ResultSrv");
    return;
  }
  robot_control_cmd_lcmt lcm_cmd;
  if (motion_id_map_.empty()) {
    ERROR("MotionIdMap empty");
    return;
  }
  lcm_cmd.mode = motion_id_map_.at(request->motion_id).map.front();
  lcm_cmd.gait_id = motion_id_map_.at(request->motion_id).map.back();
  lcm_cmd.contact = request->contact == 0 ? 15 : request->contact;
  lcm_cmd.value = request->value;
  lcm_cmd.duration = request->duration;
  GET_VALUE(request->step_height, lcm_cmd.step_height, 2, "step_height");
  GET_VALUE(request->vel_des, lcm_cmd.vel_des, 3, "vel_des");
  GET_VALUE(request->rpy_des, lcm_cmd.rpy_des, 3, "rpy_des");
  GET_VALUE(request->pos_des, lcm_cmd.pos_des, 3, "pos_des");
  GET_VALUE(request->ctrl_point, lcm_cmd.ctrl_point, 3, "ctrl_point");
  GET_VALUE(request->acc_des, lcm_cmd.acc_des, 6, "acc_des");
  GET_VALUE(request->foot_pose, lcm_cmd.foot_pose, 6, "foot_pose");
  std::unique_lock<std::mutex> lk(lcm_write_mutex_);
  lcm_cmd_ = lcm_cmd;
  lcm_cmd_.life_count = life_count_++;
  lcm_publish_instance_->publish(kLCMActionControlChannel, &lcm_cmd_);
  lk.unlock();
  lcm_cmd_init_ = true;
  INFO(
    "ResultCmd: %d, %d, %d, %d", lcm_cmd_.mode, lcm_cmd_.gait_id, lcm_cmd_.life_count,
    lcm_cmd_.duration);
  if (toml_log_func_) {
    toml_log_func_(lcm_cmd_);
  }
}


void MotionAction::Execute(const MotionSequenceShowSrv::Request::SharedPtr request)
{
  if (!ins_init_) {
    ERROR("MotionAction has not been initialized when execute QueueSrv");
    return;
  }
  if (motion_id_map_.empty()) {
    return;
  }
  INFO("SequenceCmd: \n%s", request->pace_toml.c_str());
  if (!SequenceDefImpl(request->pace_toml)) {
    ERROR("SequenceCmd(%d) transmission error", request->motion_id);
  }
}

void MotionAction::Execute(const MotionQueueCustomSrv::Request::SharedPtr request)
{
  if (!ins_init_) {
    ERROR("MotionAction has not been initialized when execute QueueSrv");
    return;
  }
  if (motion_id_map_.empty()) {
    return;
  }
  for (auto cmd : request->cmds) {
    robot_control_cmd_lcmt lcm_cmd;
    lcm_cmd.mode = cmd.mode;
    lcm_cmd.gait_id = cmd.gait_id;
    // lcm_cmd.life_count = life_count_++;
    lcm_cmd.contact = cmd.contact;
    GET_VALUE(cmd.step_height, lcm_cmd.step_height, 2, "step_height");
    GET_VALUE(cmd.vel_des, lcm_cmd.vel_des, 3, "vel_des");
    GET_VALUE(cmd.rpy_des, lcm_cmd.rpy_des, 3, "rpy_des");
    GET_VALUE(cmd.pos_des, lcm_cmd.pos_des, 3, "pos_des");
    GET_VALUE(cmd.ctrl_point, lcm_cmd.ctrl_point, 3, "ctrl_point");
    GET_VALUE(cmd.acc_des, lcm_cmd.acc_des, 6, "acc_des");
    GET_VALUE(cmd.foot_pose, lcm_cmd.foot_pose, 6, "foot_pose");
    lcm_cmd.value = cmd.value;
    lcm_cmd.duration = cmd.duration;
    std::unique_lock<std::mutex> lk(lcm_write_mutex_);
    lcm_cmd_ = lcm_cmd;
    lcm_cmd_.life_count = life_count_++;
    lcm_publish_instance_->publish(kLCMActionControlChannel, &lcm_cmd_);
    lk.unlock();
    lcm_cmd_init_ = true;
    INFO(
      "CustomCmd: %d, %d, %d, %d", lcm_cmd_.mode, lcm_cmd_.gait_id, lcm_cmd_.life_count,
      lcm_cmd_.duration);
    if (toml_log_func_) {
      toml_log_func_(lcm_cmd_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool MotionAction::ParseMotionIdMap()
{
  std::string motion_id_map_config = ament_index_cpp::get_package_share_directory("motion_action") +
    "/preset/" + "motion_id_map.toml";
  toml::value motion_ids;
  if (!cyberdog::common::CyberdogToml::ParseFile(motion_id_map_config, motion_ids)) {
    FATAL("Cannot parse %s", motion_id_map_config.c_str());
    return false;
  }
  if (!motion_ids.is_table()) {
    FATAL("Toml format error");
    exit(-1);
  }
  toml::value values;
  cyberdog::common::CyberdogToml::Get(motion_ids, "motion_ids", values);
  std::map<int32_t, MotionIdMap> motion_id_map;
  for (size_t i = 0; i < values.size(); i++) {
    auto value = values.at(i);
    int32_t motion_id;
    MotionIdMap motion_id_map;
    GET_TOML_VALUE(value, "motion_id", motion_id);
    GET_TOML_VALUE(value, "map", motion_id_map.map);
    GET_TOML_VALUE(value, "pre_motion", motion_id_map.pre_motion);
    GET_TOML_VALUE(value, "post_motion", motion_id_map.post_motion);
    GET_TOML_VALUE(value, "min_exec_time", motion_id_map.min_exec_time);
    motion_id_map_.emplace(motion_id, motion_id_map);
  }
  return true;
}

// bool MotionAction::ParseElecSkin()
// {
//   std::string elec_skin_config = ament_index_cpp::get_package_share_directory("motion_action") +
//     "/preset/" + "elec_skin.toml";
//   toml::value elec_skin_value;
//   if (!cyberdog::common::CyberdogToml::ParseFile(elec_skin_config, elec_skin_value)) {
//     FATAL("Cannot parse %s", elec_skin_config.c_str());
//     return false;
//   }
//   // if (!motion_ids.is_table()) {
//   //   FATAL("Toml format error");
//   //   exit(-1);
//   // }
//   // toml::value values;
//   int8_t default_color = 0;
//   int8_t start_direction = 0;
//   cyberdog::common::CyberdogToml::Get(elec_skin_value, "default_color", default_color);
//   cyberdog::common::CyberdogToml::Get(elec_skin_value, "start_direction", start_direction);
//   cyberdog::common::CyberdogToml::Get(elec_skin_value, "gradual_duration", gradual_duration_);
//   cyberdog::common::CyberdogToml::Get(elec_skin_value,
//   "stand_gradual_duration_", stand_gradual_duration_);
//   cyberdog::common::CyberdogToml::Get(elec_skin_value,
//   "twink_gradual_duration_", twink_gradual_duration_);
//   cyberdog::common::CyberdogToml::Get(elec_skin_value,
//   "random_gradual_duration_", random_gradual_duration_);

//   INFO("Default color: %d", default_color);
//   INFO("Start direction: %d", start_direction);
//   INFO("Gradual duration: %d", gradual_duration_);
//   INFO("StandGradual duration: %d", stand_gradual_duration_);
//   INFO("TwinkGradual duration: %d", twink_gradual_duration_);
//   INFO("RandomGradual duration: %d", random_gradual_duration_);
//   if (default_color == 0) {
//     change_dir_.push_back(PositionColorChangeDirection::PCCD_WTOB);
//     change_dir_.push_back(PositionColorChangeDirection::PCCD_BTOW);
//   } else {
//     change_dir_.push_back(PositionColorChangeDirection::PCCD_BTOW);
//     change_dir_.push_back(PositionColorChangeDirection::PCCD_WTOB);
//   }
//   if (start_direction == 0) {
//     start_dir_ = PositionColorStartDirection::PCSD_FRONT;
//   } else {
//     start_dir_ = PositionColorStartDirection::PCSD_BACK;
//   }
//   return true;
// }

bool MotionAction::Init(
  const std::string & publish_url, const std::string & subscribe_url)
{
  if (!ParseMotionIdMap()) {
    ERROR("Fail to parse MotionID");
    return false;
  }
  lcm_publish_duration_ = 1 / static_cast<float>(kActionLcmPublishFrequency) * 1000;
  lcm_publish_instance_ = std::make_shared<lcm::LCM>(publish_url);
  lcm_subscribe_instance_ = std::make_shared<lcm::LCM>(subscribe_url);
  lcm_subscribe_instance_->subscribe(
    kLCMActionResponseChannel,
    &MotionAction::ReadActionResponseLcm, this);
  if (!lcm_subscribe_instance_->good()) {
    ERROR("MotionAction read lcm initialized error");
    return false;
  }
  if (!lcm_publish_instance_->good()) {
    ERROR("MotionAction write lcm initialized error");
    return false;
  }
  control_thread_ = std::thread(&MotionAction::WriteLcm, this);
  control_thread_.detach();
  response_thread_ =
    std::thread(
    [this]() {
      while (rclcpp::ok()) {
        if (0 == this->lcm_subscribe_instance_->handleTimeout(kAcitonLcmReadTimeout)) {
          this->lcm_ready_ = false;
          if (state_ == MotionMgrState::kActive ||
          state_ == MotionMgrState::kSetup ||
          state_ == MotionMgrState::kSelfCheck)
          {
            ERROR("Cannot read LCM from MR813");
          }
        } else {
          this->lcm_ready_ = true;
        }
      }
    });
  response_thread_.detach();
  lcm_recv_subscribe_instance_ = std::make_shared<lcm::LCM>(publish_url);
  if (!lcm_recv_subscribe_instance_->good()) {
    ERROR("MotionAction read recv lcm initialized error");
    return false;
  }
  lcm_recv_subscribe_instance_->subscribe(
    kLCMActionSeqDefResultChannel,
    &MotionAction::ReadSeqDefResultLcm, this);
  std::thread{[this]() {
      while (rclcpp::ok()) {
        while (0 == this->lcm_recv_subscribe_instance_->handle()) {}
      }
    }}.detach();

  lcm_state_estimator_subscribe_instance_ = std::make_shared<lcm::LCM>(
    "udpm://239.255.76.67:7669?ttl=255");
  if (!lcm_state_estimator_subscribe_instance_->good()) {
    ERROR("MotionAction read robot state lcm initialized error");
    return false;
  }
  lcm_state_estimator_subscribe_instance_->subscribe(
    "state_estimator",
    &MotionAction::ReadStateEstimatorLcm, this);
  std::thread{[this]() {
      while (rclcpp::ok()) {
        while (0 == this->lcm_state_estimator_subscribe_instance_->handle()) {}
      }
    }}.detach();

  elec_skin_manager_ = std::make_shared<SkinManagerNode>("skin_manager");
  std::thread{[]() {
      std::system(
        "lcm-logger --channel=external_imu --invert-channels --rotate=5 --split-mb=5 /SDCARD/lcm_log/logfile --quiet");  // NOLINT
    }}.detach();
  // ParseElecSkin();
  // leg_map.emplace(0, std::vector<PositionSkin>{PositionSkin::PS_RFLEG, PositionSkin::PS_FRONT});
  // leg_map.emplace(1, std::vector<PositionSkin>{PositionSkin::PS_LFLEG, PositionSkin::PS_BODYL});
  // leg_map.emplace(2, std::vector<PositionSkin>{PositionSkin::PS_RBLEG, PositionSkin::PS_BODYR});
  // leg_map.emplace(3, std::vector<PositionSkin>{PositionSkin::PS_LBLEG, PositionSkin::PS_BODYM});
  ins_init_ = true;
  return true;
}

void MotionAction::ReadStateEstimatorLcm(
  const lcm::ReceiveBuffer *, const std::string &,
  const state_estimator_lcmt * msg)
{
  if (!ins_init_) {
    return;
  }
  // if (!align_contact_) {
  //   return;
  // }
  // if (!move_skin_) {
  //   return;
  // }
  auto contact = std::vector<float>(4, 0);
  for (uint8_t i = 0; i < 4; i++) {
    contact[i] = msg->contactEstimate[i];
  }
  elec_skin_manager_->ShowMoveElecSkin(contact);

  // static auto last_contact = std::vector<uint8_t>(4, 0);
  // auto contact = std::vector<uint8_t>(4, 0);
  // // INFO("%f, %f, %f, %f", msg->contactEstimate[0],msg->contactEstimate[1],
  // msg->contactEstimate[2],msg->contactEstimate[3]);
  // for(uint8_t i = 0; i < 4; ++i) {
  //   contact[i] = msg->contactEstimate[i] > 0 ? 1 : 0;
  //   if (contact[i] == last_contact[i]) {
  //     continue;
  //   }
  //   last_contact[i] = contact[i];
  //   if (contact[i] == 1) {
  //     WARN("Leg %d liftdown", i);
  //     for (auto p : leg_map[i]) {
  //       elec_skin_->PositionContril(
  //         p,
  //         change_dir_.front(),
  //         start_dir_,
  //         gradual_duration_);
  //     }
  //   } else {
  //     WARN("Leg %d liftup", i);
  //     for (auto p : leg_map[i]) {
  //       elec_skin_->PositionContril(
  //         p,
  //         change_dir_.back(),
  //         start_dir_,
  //         gradual_duration_);
  //     }
  //   }

  // }
  // INFO("%d, %d, %d, %d", contact[0], contact[1], contact[2], contact[3]);
}


bool MotionAction::SelfCheck()
{
  INFO("MR813 LCM: %s", lcm_ready_ ? "Ready" : "Offline");
  return lcm_ready_;
}

void MotionAction::RegisterFeedback(
  std::function<void(MotionStatusMsg::SharedPtr)> feedback)
{
  feedback_func_ = feedback;
}

void MotionAction::RegisterTomlLog(
  std::function<void(const robot_control_cmd_lcmt &)> toml_log)
{
  toml_log_func_ = toml_log;
}

void MotionAction::ReadActionResponseLcm(
  const lcm::ReceiveBuffer *, const std::string &,
  const robot_control_response_lcmt * msg)
{
  // TODO(harvey):
  if (show_) {
    INFO(
      "bar:%d, mod:%d, gid:%d, sws:%d, fer:%d", msg->order_process_bar, msg->mode, msg->gait_id,
      msg->switch_status, msg->footpos_error);
  }
  static auto lcm_res = std::make_shared<MotionStatusMsg>();
  if (msg->mode != last_res_mode_ || msg->gait_id != last_res_gait_id_) {
    last_res_mode_ = msg->mode;
    last_res_gait_id_ = msg->gait_id;
    if (msg->mode == 0) {
      last_motion_id_ = MotionIDMsg::ESTOP;
    } else {
      for (auto m = motion_id_map_.begin(); ; m++) {
        if (m == motion_id_map_.end()) {
          WARN_EXPRESSION(
            lcm_cmd_init_,
            "Get unkown response about motion_id, mode: %d, gait_id: %d!",
            static_cast<int>(msg->mode), static_cast<int>(msg->gait_id));
          last_motion_id_ = -1;
          break;
        }
        if (m->second.map.front() == msg->mode && m->second.map.back() == msg->gait_id) {
          last_motion_id_ = m->first;
          break;
        }
      }
    }
  }
  lcm_res->motion_id = last_motion_id_;
  lcm_res->contact = msg->contact;
  lcm_res->order_process_bar = msg->order_process_bar;
  lcm_res->switch_status = msg->switch_status;
  lcm_res->ori_error = msg->ori_error;
  lcm_res->footpos_error = msg->footpos_error;
  lcm_res->motor_error.resize(12);
  for (uint8_t i = 0; i < 12; ++i) {
    lcm_res->motor_error[i] = msg->motor_error[i];
  }
  static int8_t count = 5;
  if (count > 0) {
    INFO("feedback func: %s", feedback_func_.target_type().name());
    INFO("%d", feedback_func_ != nullptr);
    --count;
  }
  if (feedback_func_ != nullptr) {
    feedback_func_(lcm_res);
  }
}

void MotionAction::ReadSeqDefResultLcm(
  const lcm::ReceiveBuffer *, const std::string &,
  const file_recv_lcmt * msg)
{
  INFO("Get Sequence definition result: %d", msg->result);
  sequence_recv_result_ = false;
  sequence_recv_result_ = msg->result;
  if (sequence_def_result_waiting_) {
    seq_def_result_cv_.notify_one();
    sequence_def_result_waiting_ = false;
  }
}

void MotionAction::WriteLcm()
{
  while (lcm_publish_instance_->good()) {
    if (lcm_cmd_init_) {
      std::unique_lock<std::mutex> lk(lcm_write_mutex_);
      lcm_publish_instance_->publish(kLCMActionControlChannel, &lcm_cmd_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(lcm_publish_duration_));
  }
}

bool MotionAction::SequenceDefImpl(const std::string & toml_data)
{
  {
    std::unique_lock<std::mutex> lk(lcm_write_mutex_);
    file_send_lcmt msg;
    msg.data = toml_data;
    lcm_publish_instance_->publish(kLCMActionSequenceDefChannel, &msg);
  }
  std::unique_lock<std::mutex> lk(seq_def_result_mutex_);
  sequence_def_result_waiting_ = true;
  if (seq_def_result_cv_.wait_for(
      lk,
      std::chrono::milliseconds(kAcitonLcmReadTimeout + 2000)) == std::cv_status::timeout)
  {
    ERROR("Wait sequence def or trans result timeout");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  return sequence_recv_result_ == 0;
}


}  // namespace motion
}  // namespace cyberdog
