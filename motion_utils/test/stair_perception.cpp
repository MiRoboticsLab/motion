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
#include "motion_utils/stair_perception.hpp"

namespace cyberdog
{
namespace motion
{

StairPerception::StairPerception(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  pc_raw_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pc_filtered_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  ro_filter_.setRadiusSearch(0.05);
  ro_filter_.setMinNeighborsInRadius(5);

  pcl_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "head_pc",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&StairPerception::HandlePointCloud, this, std::placeholders::_1));
  pc_ro_filtered_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("ro_filtered", 1);
  state_ = State::IDLE;
  trigger_ = true;
}

void StairPerception::HandlePointCloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  INFO("----------------");
  pcl::fromROSMsg(msg, *pc_raw_);
  ro_filter_.setInputCloud(pc_raw_);
  ro_filter_.filter(*pc_filtered_);
  pcl::toROSMsg(*pc_filtered_, pc_filtered_ros_);
  pc_ro_filtered_pub_->publish(pc_filtered_ros_);
  int total_points_size = pc_filtered_->size();
  int left_point_size = 0;
  int right_point_size = 0;
  int dead_zone = 2, correction = 0;

  for (auto point : pc_filtered_->points) {
    if (point.y > 0) {
      left_point_size++;
    } else {
      right_point_size++;
    }
  }
  int diff = GetMeanDiff(left_point_size - right_point_size);
  switch (state_) {
    case State::IDLE:
      if (trigger_) {
        state_ = State::BLIND_FORWARD;
        trigger_ = false;
        INFO("Launch!");
      }
      break;

    case State::BLIND_FORWARD:
      if (total_points_size < 15) {
        INFO("Points size %d < threshold, stair not found, Blind Forward", total_points_size);
        break;
      }
      if (diff < -dead_zone + correction) {
        INFO("Turn right: %d", diff);
        state_ = State::TURN_RIGHT;
      } else if (diff > dead_zone + correction) {
        INFO("Turn left: %d", diff);
        state_ = State::TURN_LEFT;
      } else {
        INFO("Approach Directly: %d", diff);
        state_ = State::APPROACH;
      }
      break;

    case State::TURN_LEFT:
      if (diff <= dead_zone + correction) {
        INFO("Finish turning left: %d", diff);
        state_ = State::IDLE;
      }
      INFO("Turn left: %d", diff);
      break;

    case State::TURN_RIGHT:
      if (diff >= -dead_zone + correction) {
        INFO("Finish turning right: %d", diff);
        state_ = State::IDLE;
      }
      INFO("Turn right: %d", diff);
      break;

    case State::APPROACH:
      if (total_points_size > 100) {
        INFO("Stop: %d", total_points_size);
        state_ = State::IDLE;
      }
      INFO("Approaching: %d", total_points_size);
      break;

    default:
      break;
  }
}
}
}
