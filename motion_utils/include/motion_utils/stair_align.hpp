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

#include <iostream>
#include <rclcpp/node.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include "motion_utils/stair_perception.hpp"
#include "motion_action/motion_macros.hpp"
namespace cyberdog
{
namespace motion
{
class StairAlign
{

public:
  StairAlign(rclcpp::Node::SharedPtr node);
  void Spin()
  {
    rclcpp::spin(node_);
  }

private:
  void Loop();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<MotionServoCmdMsg>::SharedPtr servo_cmd_pub_;
  MotionServoCmdMsg servo_cmd_;
  std::shared_ptr<StairPerception> stair_perception_;
};  // calss StairAlign
}  // motion
}  // cyberdog
#endif // MOTION_UTILS__STAIR_ALIGN_HPP_
