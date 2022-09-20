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

#include <string>
#include <memory>
#include "motion_bridge/odom_out_publisher.hpp"

namespace cyberdog
{
namespace motion
{
OdomOutPublisher::OdomOutPublisher(const rclcpp::Node::SharedPtr node)
{
  node_ = node;
  odom_frame_ = node_->declare_parameter("odom_frame", std::string("odom"));
  base_frame_ = node_->declare_parameter("base_frame", std::string("base_link_leg"));
  map_frame_ = node_->declare_parameter("map_frame", std::string("map"));
  tf_pub_ = node->declare_parameter("tf_pub", true);
  INFO("%d", tf_pub_);
  leg_odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>(
    kBridgeOdomTopicName,
    rclcpp::SystemDefaultsQoS());
  tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  lcm_ = std::make_shared<lcm::LCM>(kLCMBirdgeSubscribeURL);
  lcm_->subscribe(kLCMBridgeOdomChannel, &OdomOutPublisher::OdomLCMCabllback, this);
  std::thread{
    [this]() {
      while (rclcpp::ok()) {
        while (0 == this->lcm_->handleTimeout(1000) && rclcpp::ok()) {
          ERROR("Cannot read LegOdomLCM from MR813");
          if (ready_publish_) {
            ready_publish_ = false;
          }
        }
        if (!ready_publish_) {
          ready_publish_ = true;
          cv_.notify_one();
        }
      }
    }
  }.detach();
  std::thread{
    [this]() {
      while (rclcpp::ok()) {
        if (!ready_publish_) {
          std::unique_lock<std::mutex> lk(mutex_);
          cv_.wait(lk);
        }
        leg_odom_publisher_->publish(odom_);
        if (this->tf_pub_) {
          tf2_broadcaster_->sendTransform(transform_stamped_);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }
  }.detach();
}

void OdomOutPublisher::OdomLCMCabllback(
  const lcm::ReceiveBuffer *, const std::string &,
  const localization_lcmt * msg)
{
  odom_.header.frame_id = transform_stamped_.header.frame_id = odom_frame_;
  odom_.header.stamp = transform_stamped_.header.stamp = node_->get_clock()->now();
  odom_.child_frame_id = transform_stamped_.child_frame_id = base_frame_;
  odom_.pose.pose.position.x = transform_stamped_.transform.translation.x = msg->xyz[0];
  odom_.pose.pose.position.y = transform_stamped_.transform.translation.y = msg->xyz[1];
  odom_.pose.pose.position.z = transform_stamped_.transform.translation.y = msg->xyz[2];
  tf2::Quaternion q;
  q.setRPY(msg->rpy[0], msg->rpy[1], msg->rpy[2]);
  odom_.pose.pose.orientation.x = transform_stamped_.transform.rotation.x = q.getX();
  odom_.pose.pose.orientation.y = transform_stamped_.transform.rotation.y = q.getY();
  odom_.pose.pose.orientation.z = transform_stamped_.transform.rotation.z = q.getZ();
  odom_.pose.pose.orientation.w = transform_stamped_.transform.rotation.w = q.getW();
  odom_.twist.twist.linear.x = msg->vxyz[0];
  odom_.twist.twist.linear.y = msg->vxyz[1];
  odom_.twist.twist.linear.z = msg->vxyz[2];
  odom_.twist.twist.angular.x = msg->omegaBody[0];
  odom_.twist.twist.angular.y = msg->omegaBody[1];
  odom_.twist.twist.angular.z = msg->omegaBody[2];
}

void OdomOutPublisher::Spin()
{
  rclcpp::spin(node_);
  rclcpp::shutdown();
}

}  // namespace motion
}  // namespace cyberdog

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("leg_odom_publisher"));
  cyberdog::motion::OdomOutPublisher lop(node);
  lop.Spin();
  return 0;
}
