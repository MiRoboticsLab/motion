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
#include "motion_utils/edge_align.hpp"

namespace cyberdog
{
namespace motion
{

EdgeAlign::EdgeAlign(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  servo_cmd_pub_ = node_->create_publisher<MotionServoCmdMsg>(kMotionServoCommandTopicName, 1);
  result_cmd_client_ = node_->create_client<MotionResultSrv>(kMotionResultServiceName);

  servo_cmd_.motion_id = MotionIDMsg::WALK_ADAPTIVELY;
  servo_cmd_.step_height = std::vector<float>{0.05, 0.05};
  servo_cmd_.value = 2;
  std::string toml_file = ament_index_cpp::get_package_share_directory("motion_utils") + "/config/edge_align.toml";
  toml::value config;
  if(!cyberdog::common::CyberdogToml::ParseFile(toml_file, config)) {
    FATAL("Cannot parse %s", toml_file.c_str());
    exit(-1);
  }
  GET_TOML_VALUE(config, "vel_x", vel_x_);
  GET_TOML_VALUE(config, "vel_omega", vel_omega_);
  GET_TOML_VALUE(config, "jump_after_align", jump_after_align_);
  // INFO("%f, %f, %d", vel_x_, vel_omega_, jump_after_align_);
  edge_perception_ = std::make_shared<EdgePerception>(node, config);
  edge_perception_->Launch();
  std::thread{&EdgeAlign::Loop, this}.detach();
}

void EdgeAlign::Loop()
{
  while(rclcpp::ok()){
    switch (edge_perception_->GetStatus())
    {
      case EdgePerception::State::IDLE:
        break;

      case EdgePerception::State::BLIND_FORWARD:
        servo_cmd_.vel_des = std::vector<float>{vel_x_, 0.0, 0.0};
        servo_cmd_pub_->publish(servo_cmd_);
        break;
      
      case EdgePerception::State::TURN_LEFT:
        servo_cmd_.vel_des = std::vector<float>{0.0, 0.0, vel_omega_};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case EdgePerception::State::TURN_RIGHT:
        servo_cmd_.vel_des = std::vector<float>{0.0, 0.0, -vel_omega_};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case EdgePerception::State::APPROACH:
        servo_cmd_.vel_des = std::vector<float>{vel_x_, 0.0, 0.0};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case EdgePerception::State::FINISH:
      {
        edge_perception_->SetStatus(EdgePerception::State::IDLE);
        servo_cmd_.cmd_type = MotionServoCmdMsg::SERVO_END;
        servo_cmd_pub_->publish(servo_cmd_);
        if(jump_after_align_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
          MotionResultSrv::Request::SharedPtr req(new MotionResultSrv::Request);
          req->motion_id = 133;
          result_cmd_client_->async_send_request(req);
        }
        break;
      }

      default:
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("edge_align");
  cyberdog::motion::EdgeAlign sa(node);
  sa.Spin();
  rclcpp::shutdown();
}
