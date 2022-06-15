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
#ifndef MOTION_ACTION__MOTION_MACROS_HPP_
#define MOTION_ACTION__MOTION_MACROS_HPP_
#include <algorithm>
#include "cyberdog_system/robot_code.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
namespace cyberdog
{
namespace motion
{

using MotionServoCmdMsg = protocol::msg::MotionServoCmd;
using LcmResponse = robot_control_response_lcmt;
using MotionResultSrv = protocol::srv::MotionResultCmd;
using MotionStatusMsg = protocol::msg::MotionStatus;
using MotionServoResponseMsg = protocol::msg::MotionServoResponse;

constexpr uint8_t ACTION_LCM_PUBLISH_FREQUENCY_ = 20;
constexpr const char * ACTION_PUBLISH_URL = "udpm://239.255.76.67:7671?ttl=255";
constexpr const char * ACTION_SUBSCRIBE_URL = "udpm://239.255.76.67:7670?ttl=255";
constexpr const char * BRIDGE_SUBSCRIBE_URL = "udpm://239.255.76.67:7667?ttl=255";

// a: src, b: des, c: size, d: description
#define GET_VALUE(a, b, c, d) \
  if (a.size() != c) { \
    DEBUG("Size of %s (%ld) is invalid, all elements will set to 0", d, a.size()); \
    for (uint8_t i = 0; i < c; ++i) { \
      b[i] = 0; \
    } \
  } else { \
    for (uint8_t i = 0; i < c; ++i) { \
      b[i] = a[i]; \
    } \
  } \

#define GET_TOML_VALUE(a, b, c) \
  if (!cyberdog::common::CyberdogToml::Get(a, b, c)) { \
    FATAL("Cannot get value %s", b); \
    exit(-1); \
  } \

#define GET_TOML_VALUE_ARR(a, b, c, d) \
  if (!cyberdog::common::CyberdogToml::Get(a, b, c)) { \
    FATAL("Cannot get value %s", b); \
    exit(-1); \
  } \
  std::copy(c.begin(), c.end(), d); \
  c.clear(); \

/**
 * @brief 震荡计数器
 *        1. 用于记录及刷新过去一段时间计数器是否被置位
 *        2. 计数器只有true / false两种状态，为最简单寄存器模型
 *        3. 重复置位为true为无效操作，不影响计数器状态
 *        4. 重复置位为false为无效操作，会得到false返回值；
 *        5. 该计数器用于检测消息频率合法性场景；
 *
 */
struct ServoClick
{
  void Tick()
  {
    data_ = true;
  }

  bool Tock()
  {
    if (data_ == false) {
      return false;
    } else {
      data_ = false;
    }
    return true;
  }

  std::atomic_bool data_ {false};
};  // struct HeartQueue

/**
 * @brief 运动模型状态记录， 后续考虑重构为Handler状态
 *
 */
enum class DecisionStatus : uint8_t
{
  kIdle = 0,
  kServoStart = 1,
  kExecuting = 2
};  // enum class DecisionStatus

// 所有的motion相关code都从300开始，该值为全局架构设计分配
int32_t constexpr BaseCode = (int32_t)system::ModuleCode::kMotion;
enum class MotionCode : int32_t
{
  kOK = 0,
  kTimeout = BaseCode + 31,
  kCheckError = BaseCode + 32,
  kWithoutStart = BaseCode + 33
};  // enum class MotionCode
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_ACTION__MOTION_MACROS_HPP_
