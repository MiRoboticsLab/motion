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
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "motion_action/motion_macros.hpp"
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
  struct ApproachThreshold
  {
    std::array<float, 2> range;
    int threshold;
  };
public:
  StairPerception(const rclcpp::Node::SharedPtr node, const toml::value& config);
  void Launch(){};
  const State & GetStatus() const { return state_; };
  inline void SetStatus(const State& state) { state_ = state; };

private:
  void HandlePointCloud(const sensor_msgs::msg::PointCloud2 & msg);
  inline int GetMeanDiff(int diff)
  {
    diffs_.emplace_back(diff);
    if (diffs_.size() > filter_size_) {
      diffs_.pop_front();
    }
    int sum = 0;
    for (auto diff : diffs_) {
      sum += diff;
    }
    return sum / filter_size_;
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
  std::vector<ApproachThreshold> approach_thresholds_;
  double radius_{0.05};
  std::string norms_frame_{"robot"}, pc_frame_{"robot"}, base_link_frame_{"robot"};
  int min_neighbors_{5};
  int filter_size_ {10};
  int orientation_dead_zone_{2}, orientation_correction_{0};
  int blind_forward_threshold_{15}, approach_threshold_{100};
  bool trigger_ {false}, orientation_filter_ {false};
};  // calss StairPerception
}  // motion
}  // cyberdog
#endif // MOTION_UTILS__STAIR_PERCEPTION_HPP_
