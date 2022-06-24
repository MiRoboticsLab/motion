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
#include "motion_bridge/imu_bridge.hpp"

namespace cyberdog
{
namespace motion
{
ImuBridge::ImuBridge(const rclcpp::Node::SharedPtr node)
{
  node_ = node;
  imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SystemDefaultsQoS());
  lcm_subscribe_instance_ = std::make_shared<lcm::LCM>(kBirdgeSubscribeURL);
  lcm_subscribe_instance_->subscribe(kBridgeImuChannel, &ImuBridge::ReadLcm, this);
  imu_ros_data_.reset(new sensor_msgs::msg::Imu);

  lcm_handle_thread_ =
    std::thread(
    [this]() {
      while (rclcpp::ok()) {
        while (0 == this->lcm_subscribe_instance_->handleTimeout(kAcitonLcmReadTimeout)) {
          ERROR("Cannot read LCM from MR813");
        }
      }
    });
  lcm_handle_thread_.detach();
}

void ImuBridge::Spin()
{
  rclcpp::spin(node_);
  rclcpp::shutdown();
}

void ImuBridge::ReadLcm(
  const lcm::ReceiveBuffer *, const std::string &,
  const microstrain_lcmt * msg)
{
  imu_ros_data_->header.frame_id = imu_frame_;
  imu_ros_data_->header.stamp = node_->get_clock()->now();
  imu_ros_data_->linear_acceleration.x = msg->acc[0];
  imu_ros_data_->linear_acceleration.y = msg->acc[1];
  imu_ros_data_->linear_acceleration.z = msg->acc[2];
  imu_ros_data_->angular_velocity.x = msg->omega[0];
  imu_ros_data_->angular_velocity.y = msg->omega[1];
  imu_ros_data_->angular_velocity.z = msg->omega[2];
  imu_ros_data_->orientation.x = msg->quat[0];
  imu_ros_data_->orientation.y = msg->quat[1];
  imu_ros_data_->orientation.z = msg->quat[2];
  imu_ros_data_->orientation.w = msg->quat[3];
  imu_pub_->publish(*imu_ros_data_);

}
}  // namespace motion
}  // namespace cyberdog

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("test_imu_bridege"));
  cyberdog::motion::ImuBridge ib(node);
  ib.Spin();
  return 0;
}
