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
#include "motion_utils/stair_align.hpp"

namespace cyberdog
{
namespace motion
{

StairAlign::StairAlign(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  servo_cmd_pub_ = node_->create_publisher<MotionServoCmdMsg>(kMotionServoCommandTopicName, 1);
  align_finish_pub_ = node_->create_publisher<std_msgs::msg::Bool>("stair_align_finished_flag", 1);
  result_cmd_client_ = node_->create_client<MotionResultSrv>(kMotionResultServiceName);
  stair_align_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "stair_align", std::bind(&StairAlign::HandleServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  servo_cmd_.motion_id = MotionIDMsg::WALK_ADAPTIVELY;
  servo_cmd_.step_height = std::vector<float>{0.05, 0.05};
  servo_cmd_.value = 2;
  align_finish_.data = false;
  
  std::string toml_file = ament_index_cpp::get_package_share_directory("motion_utils") + "/config/stair_align.toml";
  toml::value config;
  if(!cyberdog::common::CyberdogToml::ParseFile(toml_file, config)) {
    FATAL("Cannot parse %s", toml_file.c_str());
    exit(-1);
  }
  GET_TOML_VALUE(config, "vel_x", vel_x_);
  GET_TOML_VALUE(config, "vel_omega", vel_omega_);
  GET_TOML_VALUE(config, "jump_after_align", jump_after_align_);
  GET_TOML_VALUE(config, "auto_start", auto_start_);
  // INFO("%f, %f, %d", vel_x_, vel_omega_, jump_after_align_);
  stair_perception_ = std::make_shared<StairPerception>(node, config);
  std::thread{std::bind(&StairAlign::Loop, this)}.detach();
}

void StairAlign::HandleServiceCallback(
    const std_srvs::srv::Trigger_Request::SharedPtr,
    std_srvs::srv::Trigger_Response::SharedPtr)
{
  auto_start_ = true;
  cv_.notify_one();
}

void StairAlign::Loop()
{

  while(rclcpp::ok()){
    static bool task_waiting = true;
    if (task_waiting) {
      std::unique_lock<std::mutex> lk(loop_mutex_);
      cv_.wait(lk);
      task_waiting = false;
    }
    static bool perception_launched = false;
    if (!perception_launched) {
      stair_perception_->Launch(true);
      perception_launched = false;
    }
    switch (stair_perception_->GetStatus())
    {
      case StairPerception::State::IDLE:
        break;

      case StairPerception::State::BLIND_FORWARD:
        servo_cmd_.vel_des = std::vector<float>{vel_x_, 0.0, 0.0};
        servo_cmd_pub_->publish(servo_cmd_);
        break;
      
      case StairPerception::State::TURN_LEFT:
        servo_cmd_.vel_des = std::vector<float>{0.0, 0.0, vel_omega_};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case StairPerception::State::TURN_RIGHT:
        servo_cmd_.vel_des = std::vector<float>{0.0, 0.0, -vel_omega_};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case StairPerception::State::APPROACH:
        servo_cmd_.vel_des = std::vector<float>{vel_x_, 0.0, 0.0};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case StairPerception::State::FINISH:
      {
        stair_perception_->SetStatus(StairPerception::State::IDLE);
        servo_cmd_.cmd_type = MotionServoCmdMsg::SERVO_END;
        servo_cmd_pub_->publish(servo_cmd_);
        align_finish_.data = true;
        if(jump_after_align_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
          MotionResultSrv::Request::SharedPtr req(new MotionResultSrv::Request);
          req->motion_id = 126;
          result_cmd_client_->async_send_request(req);
        }
        stair_perception_->Launch(false);
        task_waiting = true;
        break;;
      }

      default:
        break;
    }
    align_finish_pub_->publish(align_finish_);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stair_align");
  cyberdog::motion::StairAlign sa(node);
  sa.Spin();
  rclcpp::shutdown();
}
