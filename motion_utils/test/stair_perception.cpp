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
  // method_type_ = node_->declare_parameter("mothod_type", pcl::SAC_RANSAC);

  pc_raw_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pc_filtered_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // pc_tmp_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // pc_norms_.reset(new pcl::PointCloud<pcl::Normal>);

  // if (apply_norms_segmentation_) {
  //   seg_norms_.setOptimizeCoefficients(true);
  //   // seg_.setModelType(pcl::SACMODEL_PLANE);
  //   seg_norms_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  //   seg_norms_.setNormalDistanceWeight(0.1);
  //   seg_norms_.setMethodType(method_type_);
  //   seg_norms_.setMaxIterations(1000);
  //   seg_norms_.setDistanceThreshold(0.02);
  // } else {
  //   seg_points_.setOptimizeCoefficients(true);
  //   seg_points_.setModelType(pcl::SACMODEL_PLANE);
  //   seg_points_.setMethodType(method_type_);
  //   seg_points_.setMaxIterations(1000);
  //   seg_points_.setDistanceThreshold(0.05);
  // }

  ro_filter_.setRadiusSearch(0.05);
  ro_filter_.setMinNeighborsInRadius(5);

  pcl_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "head_pc",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&StairPerception::HandlePointCloud, this, std::placeholders::_1));
  pc_ro_filtered_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("ro_filtered", 1);

  // pc_planes_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("points_planes", 1);
  // plane_norms_pub_ =
  //   node_->create_publisher<visualization_msgs::msg::MarkerArray>("plane_norms", 1);
  // planes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("planes_model", 1);
  // centroids_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("centroids", 1);
  // SetMarkers();
  state_ = State::IDLE;
  trigger_ = true;
}

// void StairPerception::SetMarkers()
// {
//   norm_.header.frame_id = norms_frame_;
//   norm_.ns = "norm";
//   norm_.frame_locked = true;
//   norm_.lifetime = rclcpp::Duration(1, 0);
//   norm_.action = visualization_msgs::msg::Marker::ADD;
//   norm_.type = visualization_msgs::msg::Marker::LINE_STRIP;
//   norm_.color.a = 1.0;
//   norm_.color.r = 0.0;
//   norm_.color.g = 1.0;
//   norm_.color.b = 0.0;
//   norm_.pose.orientation.w = 1;
//   norm_.scale.x = 0.01;
//   geometry_msgs::msg::Point point;
//   point.x = point.y = point.z = 0;
//   norm_.points.push_back(point);

//   plane_.header.frame_id = base_link_frame_;
//   plane_.ns = "plane";
//   plane_.frame_locked = true;
//   plane_.lifetime = rclcpp::Duration(1, 0);
//   plane_.action = visualization_msgs::msg::Marker::ADD;
//   plane_.type = visualization_msgs::msg::Marker::CUBE;
//   plane_.color.a = 0.5;
//   plane_.color.g = 1.0;
//   plane_.scale.x = 1.0;
//   plane_.scale.y = 1.0;
//   plane_.scale.z = 0.001;
// }

// float StairPerception::CaculateSlope(geometry_msgs::msg::PointStamped & end)
// {
//   if (end.point.x < -0.1) {
//     return -1.0;
//   }
//   return acos(abs(end.point.z)) / M_PI * 180.0;
// }

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
  int dead_zone = 5, correction = -2;

  for (auto point : pc_filtered_->points) {
    if (point.y > 0) {
      left_point_size++;
    } else {
      right_point_size++;
    }
  }
  int diff = left_point_size - right_point_size;
  switch (state_) {
    case State::IDLE:
      if (trigger_) {
        state_ = State::BLIND_FORWARD;
        trigger_ = false;
        INFO("Launch!");
      }
      break;

    case State::BLIND_FORWARD:
      if (total_points_size < 20) {
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
        state_ = State::APPROACH;
      }
      INFO("Turn left: %d", diff);
      break;

    case State::TURN_RIGHT:
      if (diff >= -dead_zone + correction) {
        INFO("Finish turning right: %d", diff);
        state_ = State::APPROACH;
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
