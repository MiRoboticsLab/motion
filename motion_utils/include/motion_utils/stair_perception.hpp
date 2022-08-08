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
#ifndef MOTION_UTILS__STAIR_PERCEPTION_HPP_
#define MOTION_UTILS__STAIR_PERCEPTION_HPP_

#include <iostream>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <cyberdog_common/cyberdog_log.hpp>
namespace cyberdog
{
namespace motion
{

class StairPerception
{
public:
  enum class State
  {
    IDLE,
    BLIND_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    APPROACH,
    FINISH
  };

public:
  StairPerception(const rclcpp::Node::SharedPtr node);
  void Launch(){};
  const State & GetStatus() const { return state_; };
  inline void SetStatus(const State& state) { state_ = state; };

private:
  void HandlePointCloud(const sensor_msgs::msg::PointCloud2 & msg);
  inline int GetMeanDiff(int diff)
  {
    diffs_.emplace_back(diff);
    if (diffs_.size() > size_) {
      diffs_.pop_front();
    }
    int sum = 0;
    for (auto diff : diffs_) {
      sum += diff;
    }
    return sum / size_;
  }

  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ro_filter_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_raw_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ro_filtered_;

  sensor_msgs::msg::PointCloud2 pc_filtered_ros_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_ro_filtered_pub_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  State state_;
  std::deque<int32_t> diffs_;
  double radius_;
  std::string norms_frame_{"robot"}, pc_frame_{"robot"}, base_link_frame_{"robot"};
  int min_neighbors_;
  int size_ {10};
  bool trigger_ {false}, diff_filter_ {false};
};  // calss StairPerception
}  // motion
}  // cyberdog
#endif // MOTION_UTILS__STAIR_PERCEPTION_HPP_
