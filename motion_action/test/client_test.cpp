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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <motion_action/motion_macros.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>
#include "motion_action/motion_macros.hpp"
#include <fstream>
class SimMotionClient
{
public:
  SimMotionClient(const std::string & name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name);
    motion_result_client_ = node_ptr_->create_client<protocol::srv::MotionResultCmd>(cyberdog::motion::kMotionResultServiceName);
    motion_queue_client_ = node_ptr_->create_client<protocol::srv::MotionSequence>(cyberdog::motion::kMotionSequenceServiceName);
    code_ptr_ = std::make_shared<cyberdog::motion::MCode>(cyberdog::system::ModuleCode::kMotionManager);
    map_.emplace(code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK), "kOK");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kHwLowBattery), "kHwLowBattery");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kHwMotorOffline), "kHwMotorOffline");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kComLcmTimeout), "kComLcmTimeout");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kMotionSwitchError), "kMotionSwitchError");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kMotionTransitionTimeout), "kMotionTransitionTimeout");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kMotionExecuteTimeout), "kMotionExecuteTimeout");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kMotionExecuteError), "kMotionExecuteError");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kCommandInvalid), "kCommandInvalid");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kEstop), "kEstop");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kStuck), "kStuck");
    map_.emplace(code_ptr_->GetCode(cyberdog::motion::MotionCode::kBusy), "kBusy");
  }

  void Run(int argc, char ** argv)
  {
    std::chrono::milliseconds timeout(1000);
    if (!motion_result_client_->wait_for_service(timeout) || !motion_queue_client_->wait_for_service(timeout)) {
      FATAL("Service not avalible");
      return;
    }
    cmd_preset_ = ament_index_cpp::get_package_share_directory("motion_action") + "/preset/" + argv[1] + ".toml";
    cmd_def_ = ament_index_cpp::get_package_share_directory("motion_action") + "/preset/user_gait_" + argv[1] + ".toml";
    if(std::atoi(argv[1]) < 400) {
      HandleResultCmd(argc, argv);
    } else {
      HandleSequenceCmd(argc, argv);
    }
  }

  void Spin()
  {
    // executor_->spin_once();
    rclcpp::shutdown();
  }

private:
  void HandleResultCmd(int argc, char **argv) {
    protocol::srv::MotionResultCmd::Request::SharedPtr req(new protocol::srv::MotionResultCmd::Request);
    toml::value value;
    if (!cyberdog::common::CyberdogToml::ParseFile(cmd_preset_, value)) {
      FATAL("Cannot parse %s", cmd_preset_.c_str());
      exit(-1);
    }
    // GET_TOML_VALUE(value, "motion_id", req->motion_id);
    req->motion_id = std::atoi(argv[1]);
    GET_TOML_VALUE(value, "vel_des", req->vel_des);
    GET_TOML_VALUE(value, "rpy_des", req->rpy_des);
    GET_TOML_VALUE(value, "pos_des", req->pos_des);
    GET_TOML_VALUE(value, "acc_des", req->acc_des);
    GET_TOML_VALUE(value, "ctrl_point", req->ctrl_point);
    GET_TOML_VALUE(value, "foot_pose", req->foot_pose);
    GET_TOML_VALUE(value, "step_height", req->step_height);
    GET_TOML_VALUE(value, "duration", req->duration);
    if(argc > 2) {
      req->duration = std::atoi(argv[2]);
    }
    // HandleTestCmd(msg);
    // std::shared_future<protocol::srv::MotionResultCmd::Response::SharedPtr> future_result = motion_result_client_->async_send_request(req);
    auto future_result = motion_result_client_->async_send_request(req);
    INFO(
      "MotionClient call with cmd:\n motion_id: %d\n duration: %d\n vel_des: [%.2f, %.2f, %.2f]\n rpy_des: [%.2f, %.2f, %.2f]\n pos_des: [%.2f, %.2f, %.2f]\n acc_des: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n ctrl_point: [%.2f, %.2f, %.2f]\n foot_pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n step_height: [%.2f, %.2f]\n", req->motion_id, req->duration,
      req->vel_des[0], req->vel_des[1], req->vel_des[2], req->rpy_des[0], req->rpy_des[1],
      req->rpy_des[2], req->pos_des[0], req->pos_des[1], req->pos_des[2], req->acc_des[0],
      req->acc_des[1], req->acc_des[2], req->acc_des[3], req->acc_des[4], req->acc_des[5],
      req->ctrl_point[0], req->ctrl_point[1], req->ctrl_point[2], req->foot_pose[0],
      req->foot_pose[1], req->foot_pose[2], req->foot_pose[3], req->foot_pose[4],
      req->foot_pose[5], req->step_height[0], req->step_height[1]);
    
    if(rclcpp::spin_until_future_complete(node_ptr_, future_result, std::chrono::seconds(60)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      FATAL("Service failed");
      return;
    }
    INFO("MotionClient get res:\n motion_id: %d result: %d code: %d, %s", future_result.get()->motion_id, future_result.get()->result, future_result.get()->code, map_[future_result.get()->code].c_str());
  }

  void HandleSequenceCmd(int argc, char **argv){
    (void)argc;
    (void)argv;
    protocol::srv::MotionSequence::Request::SharedPtr req(new protocol::srv::MotionSequence::Request);
    req->motion_id = protocol::msg::MotionID::SEQUENCE_CUSTOM;
    std::ifstream file(cmd_def_);
    file >> req->toml_data;
    protocol::msg::MotionCustomCmd msg;
    toml::value steps;
    if (!cyberdog::common::CyberdogToml::ParseFile(cmd_preset_, steps)) {
      FATAL("Cannot parse %s", cmd_preset_.c_str());
      exit(-1);
    }
    if(!steps.is_table()) {
      FATAL("Toml format error");
      exit(-1);
    }
    toml::value values;
    cyberdog::common::CyberdogToml::Get(steps, "step", values);
    for(size_t i = 0; i < values.size(); i++) {
      auto value = values.at(i);
      // robot_control_cmd_lcmt lcm_base;
      GET_TOML_VALUE(value, "mode", msg.mode);
      GET_TOML_VALUE(value, "gait_id", msg.gait_id);
      GET_TOML_VALUE(value, "contact", msg.contact);
      GET_TOML_VALUE(value, "life_count", msg.life_count);
      GET_TOML_VALUE(value, "value", msg.value);
      GET_TOML_VALUE(value, "duration", msg.duration);
      GET_TOML_VALUE(value, "vel_des", msg.vel_des);
      GET_TOML_VALUE(value, "rpy_des", msg.rpy_des);
      GET_TOML_VALUE(value, "pos_des", msg.pos_des);
      GET_TOML_VALUE(value, "acc_des", msg.acc_des);
      GET_TOML_VALUE(value, "ctrl_point", msg.ctrl_point);
      GET_TOML_VALUE(value, "foot_pose", msg.foot_pose);
      GET_TOML_VALUE(value, "step_height", msg.step_height);
      req->cmds.push_back(msg);
    }
    auto future_result = motion_queue_client_->async_send_request(req);
    // INFO(
    //   "MotionClient call with cmd:\n motion_id: %d\n duration: %d\n vel_des: [%.2f, %.2f, %.2f]\n rpy_des: [%.2f, %.2f, %.2f]\n pos_des: [%.2f, %.2f, %.2f]\n acc_des: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n ctrl_point: [%.2f, %.2f, %.2f]\n foot_pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n step_height: [%.2f, %.2f]\n", req->motion_id, req->duration,
    //   req->vel_des[0], req->vel_des[1], req->vel_des[2], req->rpy_des[0], req->rpy_des[1],
    //   req->rpy_des[2], req->pos_des[0], req->pos_des[1], req->pos_des[2], req->acc_des[0],
    //   req->acc_des[1], req->acc_des[2], req->acc_des[3], req->acc_des[4], req->acc_des[5],
    //   req->ctrl_point[0], req->ctrl_point[1], req->ctrl_point[2], req->foot_pose[0],
    //   req->foot_pose[1], req->foot_pose[2], req->foot_pose[3], req->foot_pose[4],
    //   req->foot_pose[5], req->step_height[0], req->step_height[1]);
    
    if(rclcpp::spin_until_future_complete(node_ptr_, future_result, std::chrono::seconds(60)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      FATAL("Service failed");
      return;
    }
    INFO("MotionClient get res:\n result: %d", future_result.get()->result);
  }
  
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_result_client_{nullptr};
  rclcpp::Client<protocol::srv::MotionSequence>::SharedPtr motion_queue_client_{nullptr};
  std::unordered_map<int, std::string> map_;
  std::string cmd_preset_, cmd_def_;
  std::shared_ptr<cyberdog::motion::MCode> code_ptr_;
  LOGGER_MINOR_INSTANCE("SimMotionClient");
};

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("test_as_publisher")
  if(argc < 2){
    FATAL("argc less than 2");
    exit(-1);
  }
  rclcpp::init(argc, argv);
  SimMotionClient smm("test_as_publisher");
  smm.Run(argc, argv);
  smm.Spin();
}
