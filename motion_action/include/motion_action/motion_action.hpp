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
#ifndef MOTION_ACTION__MOTION_ACTION_HPP_
#define MOTION_ACTION__MOTION_ACTION_HPP_
#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <iostream>
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include <unistd.h>
#include <memory>
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
using MotionServoCmdMsg = protocol::msg::MotionServoCmd;
using LcmResponse = robot_control_response_lcmt;
using MotionResultSrv = protocol::srv::MotionResultCmd;
using MotionServoResponseMsg = protocol::msg::MotionServoResponse;
using MotionStatusMsg = protocol::msg::MotionStatus;

constexpr uint8_t LCM_PUBLISH_FREQUENCY_ = 20;
constexpr const char * PUBLISH_URL = "udpm://239.255.76.67:7671?ttl=255";
constexpr const char * SUBSCRIBE_URL = "udpm://239.255.76.67:7670?ttl=255";

// a: src, b: des, c: size, d: description
#define GET_VALUE(a, b, c, d) \
          if(a.size() != c) { \
            DEBUG("Size of %s (%ld) is invalid, all elements will set to 0", d, a.size()); \
            for (uint8_t i = 0; i < c; ++i) { \
              b[i] = 0; \
            } \
          } else { \
            for (uint8_t i = 0; i < c; ++i) { \
              b[i] = a[i]; \
            } \
          } \

inline bool CompareLcmResponse(const LcmResponse& res1, const LcmResponse& res2)
{
  bool flag = true;
  for (uint8_t i = 0; i < 12; ++i) {
    flag &= (res1.motor_error[i] == res2.motor_error[i]);
  }
  return (res1.mode == res2.mode && 
          res1.gait_id == res2.gait_id &&
          res1.contact == res2.contact &&
          res1.order_process_bar == res2.order_process_bar &&
          res1.switch_status == res2.switch_status &&
          res1.ori_error == res2.ori_error &&
          res1.footpos_error == res2.footpos_error &&
          res1.motor_error && flag );
}
class MotionAction final
{
  using MotionServoCmdMsg = protocol::msg::MotionServoCmd;
  using LcmResponse = robot_control_response_lcmt;
  using MotionResultSrv = protocol::srv::MotionResultCmd;
  using MotionStatusMsg = protocol::msg::MotionStatus;

public:
  MotionAction();
  ~MotionAction();

public:
  void Execute(const MotionServoCmdMsg::SharedPtr msg);
  void Execute(const MotionResultSrv::Request::SharedPtr request);
  void RegisterFeedback(std::function<void(MotionStatusMsg::SharedPtr)> feedback);
  bool Init(
    const std::string & publish_url = PUBLISH_URL,
    const std::string & subscribe_url = SUBSCRIBE_URL);
  bool SelfCheck();

private:
  void WriteLcm();
  void ReadLcm(const lcm::ReceiveBuffer *, const std::string &,
               const robot_control_response_lcmt * msg);
  bool ParseMotionId();

private:
  std::thread control_thread_, response_thread_;
  std::function<void(MotionStatusMsg::SharedPtr)> feedback_func_;
  std::shared_ptr<lcm::LCM> lcm_publish_instance_, lcm_subscribe_instance_;
  robot_control_cmd_lcmt lcm_cmd_;
  MotionStatusMsg::SharedPtr lcm_res_;
  uint8_t lcm_publish_duration_;
  int8_t last_res_mode_, last_res_gait_id_;
  std::string lcm_publish_channel_, lcm_subscribe_channel_;
  std::map<int32_t, std::vector<int8_t>> motion_id_map_;
  bool lcm_cmd_init_, ins_init_;
  LOGGER_MINOR_INSTANCE("MotionAction");
};  // class MotionAction
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_ACTION__MOTION_ACTION_HPP_
