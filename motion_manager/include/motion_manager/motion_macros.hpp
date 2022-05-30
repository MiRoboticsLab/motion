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
#ifndef MOTION_MANAGEER__MOTION_MACROS_HPP_
#define MOTION_MANAGEER__MOTION_MACROS_HPP_
#include "cyberdog_system/robot_code.hpp"

namespace cyberdog
{
namespace motion
{
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
}  // namespaace motion
}  // namespace cyberdog
#endif  // MOTION_MANAGEER__MOTION_MACROS_HPP_