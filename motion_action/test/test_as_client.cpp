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

#include <motion_action/motion_action.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>

class SimMotionClient
{
public:
  SimMotionClient(const std::string & name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name);
    motion_result_client_ = node_ptr_->create_client<protocol::srv::MotionResultCmd>(cyberdog::motion::kMotionResultServiceName);
  }

  void Run(int argc, char ** argv)
  {
    if (!motion_result_client_->wait_for_service()) {
      FATAL("Service not avalible");
      return;
    }
    protocol::srv::MotionResultCmd::Request::SharedPtr req(new protocol::srv::MotionResultCmd::Request);
    std::string cmd_preset = ament_index_cpp::get_package_share_directory("motion_action") + "/preset/" + argv[1] + ".toml";
    toml::value value;
    if (!cyberdog::common::CyberdogToml::ParseFile(cmd_preset, value)) {
      FATAL("Cannot parse %s", cmd_preset.c_str());
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
    INFO("MotionClient get res:\n motion_id: %d result: %d code: %d", future_result.get()->motion_id, future_result.get()->result, future_result.get()->code);
  }

  void Spin()
  {
    // executor_->spin_once();
    rclcpp::shutdown();
  }

private:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_result_client_{nullptr};
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
