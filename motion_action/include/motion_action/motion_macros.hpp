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
#ifndef MOTION_ACTION__MOTION_MACROS_HPP_
#define MOTION_ACTION__MOTION_MACROS_HPP_
#include <algorithm>
#include <atomic>
#include "cyberdog_system/robot_code.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/msg/motion_id.hpp"
#include "protocol/msg/motion_code.hpp"
#include "protocol/msg/motion_sequence_param.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/srv/motion_queue_custom_cmd.hpp"
#include "protocol/srv/motion_custom_cmd.hpp"
#include "protocol/srv/motion_sequence.hpp"
#include "protocol/srv/motion_sequence_show.hpp"
#include "protocol/lcm/file_send_lcmt.hpp"
#include "protocol/lcm/file_recv_lcmt.hpp"

namespace cyberdog
{
namespace motion
{

// 所有的motion相关code都从3000开始，该值为全局架构设计分配
enum class MotionCode : int32_t
{
  kMotionSwitchEstop = 21,
  kMotionSwitchBantrans = 22,
  kMotionSwitchEdamp = 23,
  kMotionSwitchLifted = 24,
  kMotionSwitchOverHeat = 25,
  kMotionSwitchLowBat = 26,
  kMotionTransitionTimeout = 27,
  kMotionExecuteTimeout = 28,
  kMotionExecuteError = 29,

  // kCommandInvalid = 30,
  kSequenceDefError = 31,
  kHwMotorOffline = 32,
  kHwMotorOverHeat = 33,
  kHwMotorOverLoad = 34,
  kComLcmTimeout = 35,
  kEstop = 40,
  kStuck = 41,
  // kBusy = 42
  kMotionSwitchOriErr = 50,
  kMotionSwitchFootPosErr = 51,
  kMotionSwitchStandStuck = 52,
  kMotionSwitchMotorOverHeat = 53,
  kMotionSwitchMotorOverCurr = 54,
  kMotionSwitchMotorErr = 55,
  kMotionSwitchCharging = 56,
};  // enum class MotionCode

using MotionServoCmdMsg = protocol::msg::MotionServoCmd;
using LcmResponse = robot_control_response_lcmt;
using MotionResultSrv = protocol::srv::MotionResultCmd;
using MotionQueueCustomSrv = protocol::srv::MotionQueueCustomCmd;
using MotionSequenceSrv = protocol::srv::MotionSequence;
using MotionSequenceShowSrv = protocol::srv::MotionSequenceShow;
using MotionCustomSrv = protocol::srv::MotionCustomCmd;
using MotionStatusMsg = protocol::msg::MotionStatus;
using MotionServoResponseMsg = protocol::msg::MotionServoResponse;
using MotionIDMsg = protocol::msg::MotionID;
using MCode = cyberdog::system::CyberdogCode<MotionCode>;

constexpr uint8_t kActionLcmPublishFrequency = 20;
constexpr uint8_t kServoDataLostTimesThreshold = 4;
constexpr uint16_t kTransitioningTimeout = 3000;  // millisecond
constexpr uint16_t kAcitonLcmReadTimeout = 200;  // millisecond
constexpr int kMotorNormal = -2147483648;
constexpr const char * kLCMActionPublishURL = "udpm://239.255.76.67:7671?ttl=255";
constexpr const char * kLCMActionSubscibeURL = "udpm://239.255.76.67:7670?ttl=255";
constexpr const char * kLCMBirdgeSubscribeURL = "udpm://239.255.76.67:7667?ttl=255";
constexpr const char * kLCMActionControlChannel = "robot_control_cmd";
constexpr const char * kLCMActionResponseChannel = "robot_control_response";
constexpr const char * kLCMActionSequenceDefChannel = "user_gait_file";
constexpr const char * kLCMActionSeqDefResultChannel = "user_gait_result";
constexpr const char * kLCMBridgeImuChannel = "external_imu";
constexpr const char * kLCMBridgeElevationChannel = "local_heightmap";
constexpr const char * kLCMBridgeOdomChannel = "global_to_robot";
constexpr const char * kLCMBridgeFileChannel = "custom_motion";
constexpr const char * kLCMBridgeMotorChannel = "motor_temperature";
constexpr const char * kMotionServoCommandTopicName = "motion_servo_cmd";
constexpr const char * kMotionServoResponseTopicName = "motion_servo_response";
constexpr const char * kMotionResultServiceName = "motion_result_cmd";
constexpr const char * kMotionCustomServiceName = "motion_custom_cmd";
constexpr const char * kMotionQueueServiceName = "motion_queue_cmd";
constexpr const char * kMotionSequenceServiceName = "motion_sequence_cmd";
constexpr const char * kMotionQueueCommandTopicName = "motion_queue_cmd_test";
constexpr const char * kBridgeOdomTopicName = "odom_out";
constexpr const char * kMotionStatusTopicName = "motion_status";
constexpr const char * kGlobalScanTopicName = "scan";
constexpr const char * kMotionCustomCmdConfigPath = "/";
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

enum class MotionID : int32_t
{
  kEstop = 0,
  kGetDown = 101,
  kRecoveryStand = 111,
  kForceControlDefinitively = 201,
  kForceControlRelatively = 202,
  kPoseControlDefinitively = 211,
  kPoseControlRelatively = 212
};  // enmu calss MotionID

enum class MotionMgrState : uint8_t
{
  kUninit,
  kSetup,
  kTearDown,
  kSelfCheck,
  kActive,
  kDeactive,
  kProtected,
  kLowPower,
  kOTA,
  kError
};

enum class HandlerStatus : uint8_t
{
  kIdle = 0,
  kExecutingServoCmd = 1,
  kExecutingResultCmd = 2
};  // enum class HandlerStatus
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_ACTION__MOTION_MACROS_HPP_
