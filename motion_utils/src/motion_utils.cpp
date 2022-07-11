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
#include "motion_utils/motion_utils.hpp"
#include <string>
#include <memory>
#include <map>
#include <vector>

namespace cyberdog
{
namespace motion
{

MotionUtils::MotionUtils()
{
  node_ = rclcpp::Node::SharedPtr(new rclcpp::Node("motion_utils"));

  servo_cmd_pub_ = node_->create_publisher<MotionServoCmdMsg>(
    kMotionServoCommandTopicName, rclcpp::SystemDefaultsQoS());

  odom_helper_.reset(new OdomHelper(node_));
}

MotionUtils::~MotionUtils() {}

bool MotionUtils::ExecuteWalkDuration(int duration, MotionServoCmdMsg::SharedPtr msg)
{
  auto deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(duration);
  while (rclcpp::ok() && std::chrono::system_clock::now() < deadline) {
    servo_cmd_pub_->publish(*msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return true;
}

bool MotionUtils::ExecuteWalkDuration(int duration, float x_vel, float y_vel, float omega)
{
  MotionServoCmdMsg::SharedPtr msg(new MotionServoCmdMsg);
  msg->motion_id = MotionIDMsg::WALK_ADAPTIVELY;
  msg->vel_des = std::vector<float>{x_vel, y_vel, omega};
  msg->step_height = std::vector<float>{0.05, 0.05};
  return ExecuteWalkDuration(duration, msg);
}


bool MotionUtils::ExecuteWalkDistance(double distance, MotionServoCmdMsg::SharedPtr msg)
{
  odom_helper_->SetStartPoint();
  while (rclcpp::ok() && odom_helper_->GetDistance() < distance * distance) {
    servo_cmd_pub_->publish(*msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  odom_helper_->Reset();
  return true;
}

bool MotionUtils::ExecuteWalkDistance(double distance, float x_vel, float y_vel, float omega)
{
  MotionServoCmdMsg::SharedPtr msg(new MotionServoCmdMsg);
  msg->motion_id = MotionIDMsg::WALK_ADAPTIVELY;
  msg->vel_des = std::vector<float>{x_vel, y_vel, omega};
  msg->step_height = std::vector<float>{0.05, 0.05};
  return ExecuteWalkDistance(distance, msg);
}

}  // namespace motion
}  // namespace cyberdog
