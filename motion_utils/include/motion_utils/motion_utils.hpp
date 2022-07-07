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
#ifndef MOTION_UTILS__MOTION_UTILS_HPP_
#define MOTION_UTILS__MOTION_UTILS_HPP_
#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <map>
#include <vector>
#include <mutex>
#include "motion_action/motion_macros.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"
#include "protocol/lcm/robot_control_cmd_lcmt.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace cyberdog
{
namespace motion
{

class MotionUtils final
{
public:
  MotionUtils();
  ~MotionUtils();

public:
  bool ExecuteWalkDuration(MotionServoCmdMsg::SharedPtr msg, float duration);
  bool ExecuteWalkDistance(float vel_x, float vel_y, float distance);

private:

private:

  LOGGER_MINOR_INSTANCE("MotionUtils");
};  // class MotionUtils
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_UTILS__MOTION_UTILS_HPP_
