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
#include "motion_manager/motion_handler.hpp"

cyberdog::motion::MotionHandler::MotionHandler(
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr publisher)
: lcm_response_pub_(publisher)
{}

cyberdog::motion::MotionHandler::~MotionHandler()
{}

void cyberdog::motion::MotionHandler::Update()
{}

void cyberdog::motion::MotionHandler::Checkout(LcmResponse * response)
{
  (void)response;
  MotionServoResponseMsg msg;
  msg.motion_id = 8;
  msg.result = true;
  msg.code = 0;
  msg.status = 0;
  lcm_response_pub_->publish(msg);
}

// void cyberdog::motion::MotionHandler::ServoResponse()
// {

// }
