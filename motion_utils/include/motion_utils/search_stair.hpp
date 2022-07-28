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
#ifndef MOTION_UTILS__SEARCH_STAIR_HPP_
#define MOTION_UTILS__SEARCH_STAIR_HPP_

#include <iostream>
// #include <ros/ros.h>
// #include <ros/subscriber.h>
// #include <ros/publisher.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/PoseArray.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <cyberdog_common/cyberdog_log.hpp>
// #include <robot_state_aggregator/GroundSegmentationConfig.h>
// #include <dynamic_reconfigure/server.h>
// #include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
namespace cyberdog
{
namespace motion
{
class StairSearch
{

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
  StairSearch(rclcpp::Node::SharedPtr node);
  void Spin()
  {
    rclcpp::spin(node_);
  }
private:
  // void HandlePointCloud(const sensor_msgs::PointCloud2ConstPtr &msg);
  void HandlePointCloud(const sensor_msgs::msg::PointCloud2 &msg);
  // void HandleDynamicReconfigCallback(const robot_state_aggregator::GroundSegmentationConfig &config, int level);
  float CaculateSlope(geometry_msgs::msg::PointStamped &end);
  void Tick(std::string marker);

  // ros::Subscriber pc_raw_sub_;
  // ros::Publisher pc_remain_ro_filtered_pub_;
  // tf::TransformListener listener_;
  // pcl::CropBox<pcl::PointXYZ> cb_filter_;
  // pcl::VoxelGrid<pcl::PointXYZ> vg_filter_;
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> ro_filter_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_raw_;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cb_filtered_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_vg_filtered_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_tmp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ro_filtered_;
  // sensor_msgs::PointCloud2 pc_remain_ro_filtered_ros_;

  std::vector<pcl::ModelCoefficients> coefficients_;
  pcl::PointCloud<pcl::PointXYZ> pc_ground_, pc_remain_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> pc_planes_;
  geometry_msgs::msg::PoseArray centroids_;
  std::vector<geometry_msgs::msg::PointStamped> norms_end_;

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_;
  boost::shared_ptr<pcl::PointCloud<pcl::Normal>> pc_norms_;
  pcl::SACSegmentation<pcl::PointXYZ> seg_points_;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg_norms_;
  pcl::ExtractIndices<pcl::PointXYZ> extract_;
  // dynamic_reconfigure::Server<robot_state_aggregator::GroundSegmentationConfig> dync_server_;


  void SetMarkers();
  visualization_msgs::msg::Marker norm_, plane_;
  visualization_msgs::msg::MarkerArray norms_, planes_;
  // sensor_msgs::PointCloud2 pc_cb_filtered_ros_;
  sensor_msgs::msg::PointCloud2 pc_vg_filtered_ros_;
  // sensor_msgs::PointCloud2 pc_remain_ros_;
  // sensor_msgs::PointCloud2 pc_ground_ros_;
  // ros::Publisher pc_planes_pub_;
  // ros::Publisher pc_ground_pub_;
  // ros::Publisher pc_vg_filtered_pub_;
  // ros::Publisher pc_remain_pub_;
  // ros::Publisher pc_cb_filtered_pub_;
  // ros::Publisher plane_norms_pub_;
  // ros::Publisher centroids_pub_;
  // ros::Publisher planes_pub_;
  sensor_msgs::msg::PointCloud2 pc_planes_ros_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_ro_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_planes_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plane_norms_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr planes_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centroids_pub_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  // ros::Time stamp_;
  // double leaf_size_;
  // double max_ground_height_;
  // // double MeanK, Stddev;
  State state_;
  double radius_;
  double min_slope_, max_slope_;
  // double max_pc_height_;
  double max_height_slope_centroid_, zero_ground_epsilon_;
  std::string norms_frame_{"robot"}, pc_frame_{"robot"}, base_link_frame_{"robot"}; 
  int min_neighbors_;
  int method_type_;
  bool apply_norms_segmentation_{false};
  bool trigger_{false};
};  // calss StairSearch
}  // motion
}  // cyberdog
#endif // MOTION_UTILS__SEARCH_STAIR_HPP_