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
#include <vector>
#include "motion_bridge/motor_bridge.hpp"

namespace cyberdog
{
namespace motion
{
MotorBridge::MotorBridge(const rclcpp::Node::SharedPtr node)
{
  node_ = node;
  motor_temp_srv_ = node_->create_service<protocol::srv::MotorTemp>(
    "motor_temp",
    std::bind(
      &MotorBridge::HandleMotorTempCallback,
      this, std::placeholders::_1, std::placeholders::_2));
  lcm_subscribe_instance_ = std::make_shared<lcm::LCM>(kLCMActionSubscibeURL);
  lcm_subscribe_instance_->subscribe(kLCMBridgeMotorChannel, &MotorBridge::ReadLcm, this);
  lcm_handle_thread_ =
    std::thread(
    [this]() {
      while (rclcpp::ok()) {
        while (0 == this->lcm_subscribe_instance_->handleTimeout(kAcitonLcmReadTimeout)) {
          ERROR("Cannot read motor temp LCM from MR813");
        }
      }
    });
  lcm_handle_thread_.detach();
}

void MotorBridge::Spin()
{
  rclcpp::spin(node_);
  rclcpp::shutdown();
}

void MotorBridge::ReadLcm(
  const lcm::ReceiveBuffer *, const std::string &,
  const danger_states_lcmt * msg)
{
  motor_temp_ = *msg;
}

void MotorBridge::HandleMotorTempCallback(
  const protocol::srv::MotorTemp_Request::SharedPtr,
  protocol::srv::MotorTemp_Response::SharedPtr res)
{
  res->result = true;
  res->motor_temp = std::vector<float>(
    std::begin(motor_temp_.motor_temperature),
    std::end(motor_temp_.motor_temperature));
}

}  // namespace motion
}  // namespace cyberdog

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("motor_temp"));
  cyberdog::motion::MotorBridge mb(node);
  mb.Spin();
  return 0;
}
