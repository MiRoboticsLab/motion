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
#ifndef MOTION_UTILS__STAIR_ALIGN_HPP_
#define MOTION_UTILS__STAIR_ALIGN_HPP_

#include <rclcpp/node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <memory>
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "motion_utils/stair_perception.hpp"
#include "motion_action/motion_macros.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cyberdog
{
namespace motion
{
class StairAlign
{
public:
  explicit StairAlign(rclcpp::Node::SharedPtr node);
  void Spin()
  {
    rclcpp::spin(node_);
  }

private:
  void Loop(
    const std_srvs::srv::Trigger_Request::SharedPtr request,
    std_srvs::srv::Trigger_Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<MotionServoCmdMsg>::SharedPtr servo_cmd_pub_, align_status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stair_align_srv_, jump_srv_;
  rclcpp::Client<MotionResultSrv>::SharedPtr result_cmd_client_;
  MotionServoCmdMsg servo_cmd_;
  std::shared_ptr<StairPerception> stair_perception_;
  float vel_x_, vel_omega_;
  bool jump_after_align_, auto_start_;
};  // calss StairAlign
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_UTILS__STAIR_ALIGN_HPP_