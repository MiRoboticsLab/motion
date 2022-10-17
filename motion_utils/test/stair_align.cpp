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
  servo_cmd_pub_ = node_->create_publisher<MotionServoCmdMsg>(
    kMotionServoCommandTopicName, 1);
  align_finish_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "stair_align_finished_flag", 1);
  result_cmd_client_ = node_->create_client<MotionResultSrv>(
    kMotionResultServiceName);
  start_stair_align_srv_ = node->create_service<std_srvs::srv::SetBool>(
    "start_stair_align",
    std::bind(&StairAlign::HandleStartAlignCallback,
      this, std::placeholders::_1, std::placeholders::_2));
  stop_stair_align_srv_ = node->create_service<std_srvs::srv::Trigger>(
    "stop_stair_align",
    std::bind(&StairAlign::HandleStopAlignCallback,
      this, std::placeholders::_1, std::placeholders::_2));
  servo_cmd_.motion_id = MotionIDMsg::WALK_ADAPTIVELY;
  servo_cmd_.step_height = std::vector<float>{0.05, 0.05};
  servo_cmd_.value = 2;
  align_finish_.data = false;
  // params config of stair_align
  std::string toml_file = ament_index_cpp::get_package_share_directory(
    "motion_utils") + "/config/stair_align.toml";
  toml::value config;
  if(!cyberdog::common::CyberdogToml::ParseFile(toml_file, config)) {
    FATAL("Cannot parse %s", toml_file.c_str());
    exit(-1);
  }
  GET_TOML_VALUE(config, "vel_x", vel_x_);
  GET_TOML_VALUE(config, "vel_omega", vel_omega_);
  GET_TOML_VALUE(config, "jump_after_align", jump_after_align_);
  GET_TOML_VALUE(config, "auto_start", auto_start_);
  GET_TOML_VALUE(config, "is_stair_mode", is_stair_mode);
  // INFO("%f, %f, %d", vel_x_, vel_omega_, jump_after_align_);
  stair_perception_ = std::make_shared<StairPerception>(node, config);

  // parmas config of edge_align
  toml_file = ament_index_cpp::get_package_share_directory(
    "motion_utils") + "/config/edge_align.toml";
  if(!cyberdog::common::CyberdogToml::ParseFile(toml_file, config)) {
    FATAL("Cannot parse %s", toml_file.c_str());
    exit(-1);
  }
  edge_perception_ = std::make_shared<EdgePerception>(node, config);
  std::thread{std::bind(&StairAlign::Loop, this)}.detach();
}

void StairAlign::HandleStartAlignCallback(
    const std_srvs::srv::SetBool_Request::SharedPtr request,
    std_srvs::srv::SetBool_Response::SharedPtr response)
{
  // auto_start_ = true;
  if (request->data) {
    INFO("Get request to align upstair");
    is_stair_mode = true;
  } else {
    INFO("Get request to align downstair");
    is_stair_mode = false;
  }
  cv_.notify_one();
  response->success = true;
}

void StairAlign::HandleStopAlignCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  task_start_= false;
  if ( is_stair_mode ) {
    stair_perception_->Launch(false);
    stair_perception_->SetStatus(StairPerception::State::IDLE);
  }
  else {
    edge_perception_->Launch(false);
    edge_perception_->SetStatus(EdgePerception::State::IDLE);
  }
  response->success = true;
}

void StairAlign::Loop()
{
  while(rclcpp::ok()){
    if (auto_start_ && !task_start_) {
      task_start_ = true;
      if ( is_stair_mode ) {
        if (!stair_perception_->CheckLaunched()) {
          stair_perception_->Launch(true);
        }
      } else {
        if (!edge_perception_->CheckLaunched()) {
          edge_perception_->Launch(true);
        }
      }
    }
    else if (!task_start_) {
      std::unique_lock<std::mutex> lk(loop_mutex_);
      cv_.wait(lk);
      task_start_ = true;
      if ( is_stair_mode ) {
        if (!stair_perception_->CheckLaunched()) {
          stair_perception_->Launch(true);
        }
      } else {
        if (!edge_perception_->CheckLaunched()) {
          edge_perception_->Launch(true);
        }
      }
    }
    int status = is_stair_mode ? static_cast<int>(stair_perception_->GetStatus()) : static_cast<int>(edge_perception_->GetStatus());
    switch ( status )
    {
      case static_cast<int>(StairAlign::State::IDLE):
        break;

      case static_cast<int>(StairAlign::State::BLIND_FORWARD):
        servo_cmd_.vel_des = std::vector<float>{vel_x_, 0.0, 0.0};
        servo_cmd_pub_->publish(servo_cmd_);
        break;
      
      case static_cast<int>(StairAlign::State::TURN_LEFT):
        servo_cmd_.vel_des = std::vector<float>{0.0, 0.0, vel_omega_};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case static_cast<int>(StairAlign::State::TURN_RIGHT):
        servo_cmd_.vel_des = std::vector<float>{0.0, 0.0, -vel_omega_};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case static_cast<int>(StairAlign::State::APPROACH):
        servo_cmd_.vel_des = std::vector<float>{vel_x_, 0.0, 0.0};
        servo_cmd_pub_->publish(servo_cmd_);
        break;

      case static_cast<int>(StairAlign::State::FINISH):
      {
        if (is_stair_mode)
          stair_perception_->SetStatus(StairPerception::State::IDLE);
        else
          edge_perception_->SetStatus(EdgePerception::State::IDLE);
        servo_cmd_.cmd_type = MotionServoCmdMsg::SERVO_END;
        servo_cmd_pub_->publish(servo_cmd_);
        align_finish_.data = true;
        align_finish_pub_->publish(align_finish_);
        if(jump_after_align_) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
          MotionResultSrv::Request::SharedPtr req(new MotionResultSrv::Request);
          req->motion_id = is_stair_mode ? 126 : 137;
          result_cmd_client_->async_send_request(req);
        }
        if (is_stair_mode)
          stair_perception_->Launch(false);
        else
          edge_perception_->Launch(false);
        task_start_ = false;
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
