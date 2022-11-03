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
#include "motion_action/motion_action.hpp"
#include "elec_skin/elec_skin.hpp"
#include <string>
#include <memory>
#include <map>
#include <vector>
#include <fstream>

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
  robot_control_cmd_lcmt lcm_cmd;
  lcm_cmd.mode = motion_id_map_.at(msg->motion_id).map.front();
  lcm_cmd.gait_id = motion_id_map_.at(msg->motion_id).map.back();
  lcm_cmd.contact = 0;
  lcm_cmd.value = msg->value;
  lcm_cmd.duration = 0;
  GET_VALUE(msg->step_height, lcm_cmd.step_height, 2, "step_height");
  GET_VALUE(msg->vel_des, lcm_cmd.vel_des, 3, "vel_des");
  GET_VALUE(msg->rpy_des, lcm_cmd.rpy_des, 3, "rpy_des");
  GET_VALUE(msg->pos_des, lcm_cmd.pos_des, 3, "pos_des");
  GET_VALUE(msg->ctrl_point, lcm_cmd.ctrl_point, 3, "ctrl_point");
  GET_VALUE(msg->acc_des, lcm_cmd.acc_des, 6, "acc_des");
  GET_VALUE(msg->foot_pose, lcm_cmd.foot_pose, 6, "foot_pose");
  std::unique_lock<std::mutex> lk(lcm_write_mutex_);
  lcm_cmd_ = lcm_cmd;
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
  lcm_cmd.contact = 15;
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


void MotionAction::Execute(const MotionSequenceSrv::Request::SharedPtr request)
{
  if (!ins_init_) {
    ERROR("MotionAction has not been initialized when execute QueueSrv");
    return;
  }
  if (motion_id_map_.empty()) {
    return;
  }
  for (auto cmd : request->params) {
    robot_control_cmd_lcmt lcm_cmd;
    lcm_cmd.mode = motion_id_map_.at(request->motion_id).map.front();
    lcm_cmd.gait_id = motion_id_map_.at(request->motion_id).map.back();
    lcm_cmd.step_height[0] = cmd.forefoot_height;
    lcm_cmd.step_height[1] = cmd.hindfoot_height;
    lcm_cmd.vel_des[0] = cmd.twist.linear.x;
    lcm_cmd.vel_des[1] = cmd.twist.linear.y;
    lcm_cmd.vel_des[2] = cmd.twist.linear.z;
    lcm_cmd.rpy_des[0] = cmd.twist.angular.x;
    lcm_cmd.rpy_des[1] = cmd.twist.angular.y;
    lcm_cmd.rpy_des[2] = cmd.twist.angular.z;
    lcm_cmd.pos_des[0] = cmd.centroid_height.x;
    lcm_cmd.pos_des[1] = cmd.centroid_height.y;
    lcm_cmd.pos_des[2] = cmd.centroid_height.z;
    lcm_cmd.foot_pose[0] = cmd.right_forefoot.x;
    lcm_cmd.foot_pose[1] = cmd.right_forefoot.y;
    lcm_cmd.foot_pose[2] = cmd.left_forefoot.x;
    lcm_cmd.foot_pose[3] = cmd.left_forefoot.y;
    lcm_cmd.foot_pose[4] = cmd.right_hindfoot.x;
    lcm_cmd.foot_pose[5] = cmd.right_hindfoot.y;
    lcm_cmd.ctrl_point[0] = cmd.left_hindfoot.x;
    lcm_cmd.ctrl_point[1] = cmd.left_hindfoot.y;
    lcm_cmd.ctrl_point[2] = cmd.friction_coefficient;
    for (int i = 0; i < 6; ++i) {
      lcm_cmd.acc_des[i] = 1.0;
    }
    lcm_cmd.duration = cmd.duration_ms;
    std::unique_lock<std::mutex> lk(lcm_write_mutex_);
    lcm_cmd_ = lcm_cmd;
    lcm_cmd_.life_count = life_count_++;
    lcm_publish_instance_->publish(kLCMActionControlChannel, &lcm_cmd_);
    lk.unlock();
    lcm_cmd_init_ = true;
    INFO(
      "SequenceCmd: %d, %d, %d, %d", lcm_cmd_.mode, lcm_cmd_.gait_id, lcm_cmd_.life_count,
      lcm_cmd_.duration);
    if (toml_log_func_) {
      toml_log_func_(lcm_cmd_);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
        while (0 == this->lcm_subscribe_instance_->handleTimeout(kAcitonLcmReadTimeout)) {
          ERROR("Cannot read LCM from MR813");
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

  lcm_state_estimator_subscribe_instance_ = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7669?ttl=255");
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

  elec_skin_ = std::make_shared<ElecSkin>();
  position_map.emplace(0, PositionSkin::PS_RFLEG); 
  position_map.emplace(1, PositionSkin::PS_LFLEG); 
  position_map.emplace(2, PositionSkin::PS_RBLEG); 
  position_map.emplace(3, PositionSkin::PS_LBLEG); 
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
  static auto last_contact = std::vector<uint8_t>(4, 0);
  auto contact = std::vector<uint8_t>(4, 0);
  // INFO("%f, %f, %f, %f", msg->contactEstimate[0],msg->contactEstimate[1],msg->contactEstimate[2],msg->contactEstimate[3]);
  for(uint8_t i = 0; i < 4; ++i) {
    contact[i] = msg->contactEstimate[i] > 0 ? 1 : 0;
    if (contact[i] == last_contact[i]) {
      continue;
    }
    last_contact[i] = contact[i];
    if (contact[i] == 1) {
      WARN("Leg %d liftdown", i);
      elec_skin_->PositionContril(
        position_map[i],
        PositionColorChangeDirection::PCCD_WTOB,
        PositionColorStartDirection::PCSD_FRONT,
        200);
    } else {
      WARN("Leg %d liftup", i);
      elec_skin_->PositionContril(
        position_map[i],
        PositionColorChangeDirection::PCCD_BTOW,
        PositionColorStartDirection::PCSD_FRONT,
        200);
    }

  }
  // INFO("%d, %d, %d, %d", contact[0], contact[1], contact[2], contact[3]);
}


bool MotionAction::SelfCheck()
{
  return true;
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
    ERROR("Wait sequence def result timeout");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  return sequence_recv_result_ == 0;
}


}  // namespace motion
}  // namespace cyberdog
