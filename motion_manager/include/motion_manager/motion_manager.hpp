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
#ifndef MOTION_MANAGER__MOTION_MANAGER_HPP_
#define MOTION_MANAGER__MOTION_MANAGER_HPP_
#include <functional>
#include <mutex>
#include <thread>
#include <string>
#include <memory>
#include <unordered_map>
#include "pluginlib/class_loader.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/lcm/robot_control_cmd_lcmt.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"
#include "manager_base/manager_base.hpp"
#include "motion_manager/motion_decision.hpp"
#include "motion_action/motion_action.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"

namespace cyberdog
{
namespace motion
{

class MotionManager final : public machine::MachineActuator
{
public:
  explicit MotionManager(const std::string & name);
  ~MotionManager();
  bool Init();
  void Run();

private:
  int32_t OnSetUp();
  int32_t OnTearDown();
  int32_t OnSelfCheck();
  int32_t OnActive();
  int32_t OnDeActive();
  int32_t OnProtected();
  int32_t OnLowPower();
  int32_t OnOTA();
  int32_t OnError();
  void SetState(const MotionMgrState & state)
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    state_ = state;
    decision_ptr_->SetState(state_);
  }
  MotionMgrState &
  GetState()
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    return state_;
  }
  /**
   * @brief
   *
   * @param code
   * @param protected_cmd 除行走、站立、趴下、急停以外的所有动作
   * @return true
   * @return false
   */
  bool IsStateValid(int32_t & code, bool protected_cmd = true);
  void MotionServoCmdCallback(const MotionServoCmdMsg::SharedPtr msg);
  void MotionResultCmdCallback(
    const MotionResultSrv::Request::SharedPtr request,
    MotionResultSrv::Response::SharedPtr response);
  void MotionCustomCmdCallback(
    const MotionCustomSrv::Request::SharedPtr request,
    MotionCustomSrv::Response::SharedPtr response);
  void MotionQueueCmdCallback(
    const MotionQueueCustomSrv::Request::SharedPtr request,
    MotionQueueCustomSrv::Response::SharedPtr response);
  void MotionSequenceShowCmdCallback(
    const MotionSequenceShowSrv::Request::SharedPtr request,
    MotionSequenceShowSrv::Response::SharedPtr response);
  void OnlineAudioPlay(const std::string & text)
  {
    static bool playing = false;
    if (playing) {
      return;
    }
    auto request = std::make_shared<protocol::srv::AudioTextPlay::Request>();
    request->is_online = true;
    request->module_name = "Motion";
    request->text = text;
    playing = true;
    auto callback = [](rclcpp::Client<protocol::srv::AudioTextPlay>::SharedFuture future) {
        playing = false;
        INFO("Audio play result: %s", future.get()->status == 0 ? "success" : "failed");
      };
    auto future = audio_client_->async_send_request(request, callback);
    if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::timeout) {
      playing = false;
      ERROR("Cannot get response from AudioPlay");
    }
  }
  bool TryGetDownOnFsm()
  {
    auto request = std::make_shared<MotionResultSrv::Request>();
    request->motion_id = MotionIDMsg::GETDOWN;
    request->cmd_source = MotionResultSrv::Request::FSM;
    auto response = std::make_shared<MotionResultSrv::Response>();
    decision_ptr_->DecideResultCmd(request, response);
    if (response->code != code_ptr_->GetKeyCode(system::KeyCode::kOK)) {
      return false;
    }
    return true;
  }
  void Report();

private:
  std::string name_;
  std::shared_ptr<MotionDecision> decision_ptr_ {nullptr};
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub_;
  rclcpp::Subscription<MotionServoCmdMsg>::SharedPtr motion_servo_sub_ {nullptr};
  rclcpp::Service<MotionResultSrv>::SharedPtr motion_result_srv_ {nullptr};
  rclcpp::Service<MotionCustomSrv>::SharedPtr motion_custom_srv_ {nullptr};
  rclcpp::Service<MotionQueueCustomSrv>::SharedPtr motion_queue_srv_ {nullptr};
  rclcpp::Service<MotionSequenceSrv>::SharedPtr motion_sequence_srv_ {nullptr};
  rclcpp::Service<MotionSequenceShowSrv>::SharedPtr motion_sequence_show_srv_ {nullptr};
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_client_{nullptr};
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_{nullptr};
  rclcpp::CallbackGroup::SharedPtr callback_group_{nullptr};
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  std::shared_ptr<MCode> code_ptr_{nullptr};
  std::unique_ptr<cyberdog::machine::HeartBeatsActuator> heart_beats_ptr_{nullptr};
  MotionMgrState state_{MotionMgrState::kUninit};
  std::mutex status_mutex_;
  std::unordered_map<MotionMgrState, std::string> status_map_;
  std::unique_ptr<std::thread> thread_{nullptr};
  std::condition_variable msg_condition_;
  std::mutex msg_mutex_;
  int32_t code_;
  int32_t motion_id_;

};  // class MotionManager
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_MANAGER_HPP_
