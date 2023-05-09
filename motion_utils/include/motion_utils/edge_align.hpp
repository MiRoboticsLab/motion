// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef MOTION_UTILS__EDGE_ALIGN_HPP_
#define MOTION_UTILS__EDGE_ALIGN_HPP_

#include <rclcpp/node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "motion_utils/edge_perception.hpp"
#include "motion_action/motion_macros.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

namespace cyberdog
{
namespace motion
{
class EdgeAlign
{
public:
  explicit EdgeAlign(rclcpp::Node::SharedPtr node);
  void Spin()
  {
    rclcpp::spin(node_);
  }

private:
  void HandleServiceCallback(
    const std_srvs::srv::Trigger_Request::SharedPtr request,
    std_srvs::srv::Trigger_Response::SharedPtr response);
  void Loop();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<MotionServoCmdMsg>::SharedPtr servo_cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr edge_align_srv_;
  rclcpp::Client<MotionResultSrv>::SharedPtr result_cmd_client_;
  MotionServoCmdMsg servo_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr align_finish_pub_;
  std_msgs::msg::Bool align_finish_;
  std::shared_ptr<EdgePerception> edge_perception_;
  std::mutex loop_mutex_;
  std::condition_variable cv_;
  float vel_x_, vel_omega_;
  bool jump_after_align_, auto_start_;
};  // calss EdgeAlign
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_UTILS__EDGE_ALIGN_HPP_
