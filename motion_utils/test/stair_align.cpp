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
  result_cmd_client_ = node_->create_client<MotionResultSrv>(kMotionResultServiceName);
  stair_perception_ = std::make_shared<StairPerception>(node);
  stair_perception_->Launch();
  servo_cmd_.motion_id = MotionIDMsg::WALK_ADAPTIVELY;
  servo_cmd_.step_height = std::vector<float>{0.05, 0.05};
  servo_cmd_.value = 2;
  std::thread{&StairAlign::Loop, this}.detach();
}

void StairAlign::Loop()
{
  while(rclcpp::ok()){
    switch (stair_perception_->GetStatus())
    {
      case StairPerception::State::IDLE:
        break;

      case StairPerception::State::BLIND_FORWARD:
        servo_cmd_.vel_des = std::vector<float>{0.1, 0.0, 0.0};
        servo_cmd_pub_->publish(servo_cmd_);
        break;
      
      case StairPerception::State::TURN_LEFT:
        servo_cmd_.vel_des = std::vector<float>{0.0, 0.0, 0.25};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case StairPerception::State::TURN_RIGHT:
        servo_cmd_.vel_des = std::vector<float>{0.0, 0.0, -0.25};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case StairPerception::State::APPROACH:
        servo_cmd_.vel_des = std::vector<float>{0.1, 0.0, 0.0};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case StairPerception::State::FINISH:
      {
        stair_perception_->SetStatus(StairPerception::State::IDLE);
        servo_cmd_.cmd_type = MotionServoCmdMsg::SERVO_END;
        servo_cmd_pub_->publish(servo_cmd_);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        MotionResultSrv::Request::SharedPtr req(new MotionResultSrv::Request);
        req->motion_id = 126;
        result_cmd_client_->async_send_request(req);
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
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stair_align");
  cyberdog::motion::StairAlign sa(node);
  sa.Spin();
  rclcpp::shutdown();
}
