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
#include "motion_utils/search_stair.hpp"

#define CAMERA_ROTATED 0.785

namespace cyberdog
{
namespace motion
{
StairSearch::StairSearch(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  // ros::NodeHandle nh("~");
  // nh.param("leaf_size", leaf_size_, 0.02);
  // nh.param("max_ground_height", max_ground_height_, 0.02);
  // nh.param("radius", radius_, 0.05);
  // nh.param("min_neighbors", min_neighbors_, 40);
  // nh.param("min_slope", min_slope_, 10.0);
  // nh.param("max_slope", max_slope_, 20.0);
  // nh.param("zero_height_epsilon", zero_ground_epsilon_, 0.05);
  // nh.param("max_height_slope_centroid", max_height_slope_centroid_, 0.3);
  // nh.param("max_pointcloud_height", max_pc_height_, 0.3);
  // nh.param("method_type", method_type_, pcl::SAC_RANSAC);
  method_type_ =  node_->declare_parameter("mothod_type", pcl::SAC_RANSAC);
  // nh.param("apply_norms_segmentation", apply_norms_segmentation_, true);
  // nh.param<std::string>("norms_frame", norms_frame_, "virtual_imi");
  // nh.param<std::string>("pc_frame", pc_frame_, "imi");
  // nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  // ROS_INFO("height is: %f", max_ground_height_);
  // ROS_INFO("radius is: %f", radius_);
  // ROS_INFO("min_neighbors is: %d", min_neighbors_);

  pc_raw_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // pc_cb_filtered_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pc_vg_filtered_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pc_tmp_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  pc_norms_.reset(new pcl::PointCloud<pcl::Normal>);

  // listener_.waitForTransform(base_link_frame_, pc_frame_, ros::Time(0), ros::Duration(10));
  // tf::StampedTransform transform;
  // listener_.lookupTransform(base_link_frame_, pc_frame_, ros::Time(0), transform);
  // double r, p, y;
  // tf::Matrix3x3(transform.inverse().getRotation()).getRPY(r, p, y);
  // cb_filter_.setRotation(Eigen::Vector3f(r, p, y));
  // cb_filter_.setMin(Eigen::Vector4f(0, -0.4, -2.0, 1.0));
  // cb_filter_.setMax(Eigen::Vector4f(1.0, 0.4, -(transform.getOrigin().z()-max_pc_height_), 1.0));

  // vg_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

  // pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
  // ne_.setSearchMethod(kdtree);
  // ne_.setRadiusSearch(0.05);

  if(apply_norms_segmentation_)
  {
    seg_norms_.setOptimizeCoefficients(true);
    // seg_.setModelType(pcl::SACMODEL_PLANE);
    seg_norms_.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg_norms_.setNormalDistanceWeight(0.1);
    seg_norms_.setMethodType(method_type_);
    seg_norms_.setMaxIterations(1000);
    seg_norms_.setDistanceThreshold(0.02);
  }
  else
  {
    seg_points_.setOptimizeCoefficients(true);
    seg_points_.setModelType(pcl::SACMODEL_PLANE);
    seg_points_.setMethodType(method_type_);
    seg_points_.setMaxIterations(1000);
    seg_points_.setDistanceThreshold(0.05);
  }

  ro_filter_.setRadiusSearch(0.05);
  ro_filter_.setMinNeighborsInRadius(5);

  // pc_raw_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("points_raw", 1, &StairSearch::HandlePointCloud, this);
  pcl_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>("head_pc", rclcpp::SystemDefaultsQoS(), std::bind(&StairSearch::HandlePointCloud, this, std::placeholders::_1));
  // pc_remain_ro_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points_remain_ro_filtered", 1);
  pc_ro_filtered_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("ro_filtered", 1);
  // dynamic_reconfigure::Server<robot_state_aggregator::GroundSegmentationConfig>::CallbackType cb = 
  //   boost::bind(&StairSearch::HandleDynamicReconfigCallback, this, _1, _2);
  // dync_server_.setCallback(cb);

  pc_planes_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("points_planes", 1);
  plane_norms_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("plane_norms", 1);
  planes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("planes_model", 1);
  centroids_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("centroids", 1);
  SetMarkers();

#ifdef VIS
  setMarkers();
  pc_vg_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points_vg_filtered", 1);
  pc_remain_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points_remain", 1);
  pc_ground_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points_ground", 1);
  pc_planes_pub_ = nh.advertise<sensor_msgs::PointCloud2>("points_planes", 1);
  plane_norms_pub_ = nh.advertise<visualization_msgs::MarkerArray>("plane_norms", 1);
  centroids_pub_ = nh.advertise<geometry_msgs::PoseArray>("centroids", 1);
  planes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("planes", 1);
#endif
  // stamp_ = ros::Time::now();
}

// #ifdef VIS
void StairSearch::SetMarkers()
{
  norm_.header.frame_id = norms_frame_;
  norm_.ns = "norm";
  norm_.frame_locked = true;
  norm_.lifetime = rclcpp::Duration(1, 0);
  norm_.action = visualization_msgs::msg::Marker::ADD;
  norm_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  norm_.color.a = 1.0;
  norm_.color.r = 0.0;
  norm_.color.g = 1.0;
  norm_.color.b = 0.0;
  norm_.pose.orientation.w = 1;
  norm_.scale.x = 0.01;
  geometry_msgs::msg::Point point;
  point.x = point.y = point.z = 0;
  norm_.points.push_back(point);

  plane_.header.frame_id = base_link_frame_;
  plane_.ns = "plane";
  plane_.frame_locked = true;
  plane_.lifetime = rclcpp::Duration(1, 0);
  plane_.action = visualization_msgs::msg::Marker::ADD;
  plane_.type = visualization_msgs::msg::Marker::CUBE;
  plane_.color.a = 0.5;
  plane_.color.g = 1.0;
  plane_.scale.x = 1.0;
  plane_.scale.y = 1.0;
  plane_.scale.z = 0.001;
}
// #endif

void StairSearch::Tick(std::string marker)
{
  // ROS_DEBUG("now: %.2f, stamp: %.2f", ros::Time::now().toSec()*1000, stamp_.toSec()*1000);
  // ROS_DEBUG("%s costs %.2fms", marker.c_str(), (ros::Time::now()-stamp_).toSec()*1000);
  // stamp_ = ros::Time::now();
}

float StairSearch::CaculateSlope(geometry_msgs::msg::PointStamped &end)
{
  if (end.point.x < -0.1)
    return -1.0;
  return acos(abs(end.point.z))/M_PI*180.0;
}

// void StairSearch::HandleDynamicReconfigCallback(const robot_state_aggregator::GroundSegmentationConfig &config, int level)
// {
//   if(leaf_size_ != config.leaf_size)
//   {
//     leaf_size_ = config.leaf_size;
//     ROS_INFO("leaf size reconfigured: %f", leaf_size_);
//     vg_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
//   }
//   if (max_ground_height_ != config.max_ground_height)
//   {
//     max_ground_height_ = config.max_ground_height;
//     ROS_INFO("height reconfigured: %f", max_ground_height_);
//     seg_norms_.setDistanceThreshold(max_ground_height_);
//     seg_points_.setDistanceThreshold(max_ground_height_);
//   }
//   if(radius_ != config.radius)
//   {
//     radius_ = config.radius;
//     ROS_INFO("radius reconfigured: %f", radius_);
//     ro_filter_.setRadiusSearch(radius_);
//   }
//   if (min_neighbors_ != config.min_neighbors)
//   { 
//     min_neighbors_ = config.min_neighbors;
//     ROS_INFO("min neighbors reconfigured: %d", min_neighbors_);
//     ro_filter_.setMinNeighborsInRadius(min_neighbors_);
//   }
  
// }

void StairSearch::HandlePointCloud(const sensor_msgs::msg::PointCloud2 &msg)
{
  INFO("----------------");
  // auto begin=ros::Time::now();
  // ROS_DEBUG("handling %d msg", msg->header.seq);
  pcl::fromROSMsg(msg, *pc_raw_);
  // Tick("get raw pc");
  // pc_raw_->header.stamp = pcl_conversions::toPCL(ros::Time().now());
  // cb_filter_.setInputCloud(pc_raw_);
  // cb_filter_.filter(*pc_cb_filtered_);
  // Tick("get crop box pointcloud");

  // vg_filter_.setInputCloud(pc_cb_filtered_);
  // vg_filter_.filter(*pc_vg_filtered_);
  // Tick("get voxel grid pointcloud");
  pc_vg_filtered_ = pc_raw_;
  // ro_filter_.setInputCloud(pc_raw_);
  // ro_filter_.filter(*pc_vg_filtered_);
  // pcl::toROSMsg(*pc_vg_filtered_, pc_vg_filtered_ros_);
  // pc_ro_filtered_pub_->publish(pc_vg_filtered_ros_);
#ifdef VIS
  pcl::toROSMsg(*pc_cb_filtered_, pc_cb_filtered_ros_);
  pc_cb_filtered_pub_.publish(pc_cb_filtered_ros_);
  pcl::toROSMsg(*pc_vg_filtered_, pc_vg_filtered_ros_);
  pc_vg_filtered_pub_.publish(pc_vg_filtered_ros_);
#endif
  // Tick("publish cb and vg pc");
  // pc_ground_.header = pc_remain_.header = pc_raw_->header;
  centroids_.header.frame_id = base_link_frame_;
  centroids_.poses.clear();
  // pc_ground_.clear();
  pc_remain_.clear();
  pc_planes_.clear();
  coefficients_.clear();
  norms_end_.clear();
  pc_norms_->clear();
  // Tick("clears");
  int i = 0, nr_points = (int)pc_vg_filtered_->points.size();
  pcl::ModelCoefficients model_coeff;
  pcl::PointCloud<pcl::PointXYZ> pc_vg_filtered = *pc_vg_filtered_;
  pcl::PointIndices inliers;
  // INFO("points size: %ld", pc_vg_filtered.points.size());
  Tick("cp vg pointcloud");
  while (pc_vg_filtered_->points.size() > 0.1 * nr_points)
  {
    if(apply_norms_segmentation_)
    {
      ne_.setInputCloud(pc_vg_filtered_);
      ne_.compute(*pc_norms_);
      Tick(std::to_string(i) + " compute norms");
      seg_norms_.setInputCloud(pc_vg_filtered_);
      seg_norms_.setInputNormals(pc_norms_);
      seg_norms_.segment(inliers, model_coeff);
      Tick(std::to_string(i) + " segmentation");
    }
    else
    {
      seg_points_.setInputCloud(pc_vg_filtered_);
      seg_points_.segment(inliers, model_coeff);
      Tick(std::to_string(i) + " segmentation");
    }
    if (inliers.indices.size() == 0)
    {
      INFO("Could not estimate a planar model for the given dataset.");
      break;
    }
    extract_.setInputCloud(pc_vg_filtered_);
    extract_.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
    extract_.setNegative(false);
    extract_.filter(*pc_tmp_);
    coefficients_.push_back(model_coeff);
    pc_planes_.push_back(*pc_tmp_);

    pcl::PointXYZ point;
    pcl::computeCentroid(*pc_tmp_, point);
    INFO("%f, %f, %f", point.x, point.y, point.z);
    // geometry_msgs::PoseStamped centroid;
    geometry_msgs::msg::PoseStamped centroid;
    centroid.header.frame_id = pc_frame_;
    centroid.pose.position.x = point.x;
    centroid.pose.position.y = point.y;
    centroid.pose.position.z = point.z;
    // tf::Quaternion q;
    tf2::Quaternion q;
    float x = model_coeff.values[0];
    float y = model_coeff.values[1];
    float z = model_coeff.values[2];
    q.setRPY(-asin(y), atan2(x, z), 0);
    centroid.pose.orientation.x = q.x();
    centroid.pose.orientation.y = q.y();
    centroid.pose.orientation.z = q.z();
    centroid.pose.orientation.w = q.w();

    // listener_.transformPose(base_link_frame_, centroid, centroid);
    centroids_.poses.push_back(centroid.pose);
    // geometry_msgs::PointStamped end;
    geometry_msgs::msg::PointStamped end;
    end.header.frame_id = pc_frame_;
    end.point.x = model_coeff.values[0];
    end.point.y = model_coeff.values[1];
    end.point.z = model_coeff.values[2];
    // listener_.transformPoint(norms_frame_, end, end);
    end.header.frame_id = norms_frame_;
    norms_end_.push_back(end);

    extract_.setNegative(true);
    extract_.filter(*pc_tmp_);

    pc_vg_filtered_.swap(pc_tmp_);
    i++;
  }
  Tick("get all planes");
  if(pc_planes_.size() == 0)
  {
    INFO("No plane found");
    return;
  }
  INFO("plane size: %ld", pc_planes_.size());
  
  int j=0;
  std::set<int> ground_ind;
  for(; j<int(pc_planes_.size()); j++)
  {
    INFO("checking %d plane size: %ld, %.2f, %.2f, %.2f", j, pc_planes_[j].points.size(), coefficients_[j].values[0], coefficients_[j].values[1], coefficients_[j].values[2]);

    float slope = CaculateSlope(norms_end_[j]);
    INFO("%d, centroid: %f, slope: %f", j, centroids_.poses[j].position.z, slope);
    if(centroids_.poses[j].position.z < -zero_ground_epsilon_)
      continue;
    if(slope<0)
      continue;
    if (centroids_.poses[j].position.z > zero_ground_epsilon_ && slope < min_slope_)
      continue;
    if (centroids_.poses[j].position.z < max_height_slope_centroid_&& slope < max_slope_)
    {
      INFO("will use %d plane as ground", j);
      pc_ground_ += pc_planes_[j];
      ground_ind.insert(j);
    }
  }
  Tick("get ground");
  INFO("ground size: %ld", pc_ground_.points.size());
  

  // if (!pc_ground_.points.empty())
  // {
  //   for (int jj = 0; jj < pc_planes_.size(); jj++)
  //   {
  //     if (ground_ind.find(jj)!=ground_ind.end())
  //       continue;
  //     pc_remain_ += pc_planes_[jj];
  //   }
  //   pc_remain_ += *pc_vg_filtered_;
    
  //   for(int j : ground_ind)
  //   {
  //     for (auto it = pc_remain_.points.begin(); it != pc_remain_.points.end();)
  //     {
  //       float val = it->x * coefficients_[j].values[0] + it->y * coefficients_[j].values[1] + it->z * coefficients_[j].values[2] + coefficients_[j].values[3];
  //       if (val > 0)
  //         it = pc_remain_.points.erase(it);
  //       else
  //         it++;
  //     }
  //   }
  //   INFO("remain size: %ld", pc_remain_.points.size());
  //   pc_remain_.width = pc_remain_.points.size();
  //   if(pc_remain_.points.size() != 0)
  //   {
  //     // ro_filter_.setInputCloud(pc_remain_.makeShared());
  //     ro_filter_.filter(pc_remain_ro_filtered_);
  //     INFO("remain filtered size: %ld", pc_remain_ro_filtered_.points.size());

  //     pcl::toROSMsg(pc_remain_ro_filtered_, pc_remain_ro_filtered_ros_);
  //     pc_remain_ro_filtered_pub_.publish(pc_remain_ro_filtered_ros_);
  //   }
  //   else
  //     INFO("No remain");
  //   Tick("get remain ro filtered");
  // }
  // else
  //   ROS_DEBUG("No ground found");
  
// #ifdef VIS
  // pcl::toROSMsg(pc_ground_, pc_ground_ros_);
  // pc_ground_pub_.publish(pc_ground_ros_);
  // pcl::toROSMsg(pc_remain_, pc_remain_ros_);
  // pc_remain_pub_.publish(pc_remain_ros_);

  pcl::PointCloud<pcl::PointXYZRGB> pc_planes;
  pc_planes.header = pc_raw_->header;
  norms_.markers.clear();
  norm_.header.stamp = node_->get_clock()->now();
  planes_.markers.clear();
  plane_.header.stamp = node_->get_clock()->now();
  pcl::PointCloud<pcl::PointXYZRGB> pc_planes_tmp;
  for (int ii = 0; ii < pc_planes_.size(); ii++)
  {
    pcl::copyPointCloud(pc_planes_[ii], pc_planes_tmp);
    for (auto &point : pc_planes_tmp)
    {
      point.r = 80 * (ii + 1) % 255;
      point.g = 240 * (ii + 1) % 255;
      point.b = 160 * (ii + 1) % 255;
    }
    pc_planes += pc_planes_tmp;
    if (norm_.points.size() > 1)
      norm_.points.pop_back();
    norm_.points.push_back(norms_end_[ii].point);
    norm_.id = ii;
    norms_.markers.push_back(norm_);
    plane_.pose = centroids_.poses[ii];
    plane_.id = ii;
    planes_.markers.push_back(plane_);
  }
  pcl::toROSMsg(pc_planes, pc_planes_ros_);
  pc_planes_pub_->publish(pc_planes_ros_);
  plane_norms_pub_->publish(norms_);
  centroids_pub_->publish(centroids_);
  planes_pub_->publish(planes_);
  Tick("publish all visualization topics");
// #endif
  // ROS_DEBUG("whole callback cost %.2fms", (ros::Time::now()-begin).toSec()*1000);
}
}
}

int main(int argc, char **argv)
{
  // ros::init(argc, argv, "ground_state_aggregator");
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("stair_search");
  cyberdog::motion::StairSearch ss(node);
  ss.Spin();
  rclcpp::shutdown();
}
