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
#ifndef ELEC_SKIN__ELEC_SKIN_HPP_
#define ELEC_SKIN__ELEC_SKIN_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "embed_protocol/embed_protocol.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

#define EVM cyberdog::embed

namespace cyberdog
{
namespace motion
{

typedef struct _can_data
{
  uint8_t data0;
  uint8_t data1;
  uint8_t data2;
  uint8_t data3;
} can_data;

enum class ModelSwitch : uint8_t
{
  MS_FLASH = 0,    // 闪烁
  MS_WAVEF = 1,    // 动画前向后变
  MS_RANDOM = 2,   // 随机
  MS_WAVEB = 3,    // 动画后向前变
  MS_CONTROL = 4,  // 上位机实时控制模式
};  // enum class PositionSkin

enum class PositionSkin : uint8_t
{
  PS_BODYM = 0,  // 背部
  PS_LBLEG = 1,  // 左后腿
  PS_BODYL = 2,  // 左侧
  PS_LFLEG = 3,  // 左前腿
  PS_FRONT = 4,  // 前胸
  PS_RFLEG = 5,  // 右前腿
  PS_BODYR = 6,  // 右侧
  PS_RBLEG = 7,  // 右后腿
};  // enum class PositionSkin

enum class PositionColorChangeDirection : uint8_t
{
  PCCD_WTOB = 0,   // 白变黑
  PCCD_BTOW = 1,   // 黑变白
};  // enum class PositionColorChangeDirection

enum class PositionColorStartDirection : uint8_t
{
  PCSD_FRONT = 0,   // 前方先变色
  PCSD_BACK  = 1,   // 后方先变色
};  // enum class PositionColorStartDirection

class ElecSkin final
{
public:
  ElecSkin()
  : ms_(ModelSwitch::MS_CONTROL)
  {
    std::string path = ament_index_cpp::get_package_share_directory("elec_skin") +
      "/toml_config/elec_skin.toml";
    ptr_can_protocol_ = std::make_shared<EVM::Protocol<can_data>>(path, false);
    ptr_can_protocol_->SetDataCallback(
      std::bind(
        &ElecSkin::recv_can_callback, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  ~ElecSkin()
  {
  }

  void ModelControl(ModelSwitch ms, uint16_t milsecs)
  {
    ms_ = ms;
    uint8_t wave_mode = static_cast<uint8_t>(ms_);
    uint8_t wave_cycle_time_low = *(reinterpret_cast<uint8_t *>(&milsecs));
    uint8_t wave_cycle_time_high = *(reinterpret_cast<uint8_t *>(&milsecs) + 1);
    ptr_can_protocol_->Operate(
      "model_on",
      std::vector<uint8_t>{wave_mode, wave_cycle_time_low, wave_cycle_time_high});
  }

  void PositionContril(
    PositionSkin ps, PositionColorChangeDirection pccd,
    PositionColorStartDirection pcsd, uint16_t milsecs)
  {
    if (ms_ != ModelSwitch::MS_CONTROL) {
      ModelControl(ModelSwitch::MS_CONTROL, milsecs);
    }
    uint8_t change_direction = static_cast<uint8_t>(pccd);
    uint8_t start_direction = static_cast<uint8_t>(pcsd);
    uint8_t wave_cycle_time_low = *(reinterpret_cast<uint8_t *>(&milsecs));
    uint8_t wave_cycle_time_high = *(reinterpret_cast<uint8_t *>(&milsecs) + 1);
    ptr_can_protocol_->Operate(
      control_dog_action_map.at(ps),
      std::vector<uint8_t>{change_direction, start_direction,
        wave_cycle_time_low, wave_cycle_time_high});
  }

private:
  void recv_can_callback(std::string & name, std::shared_ptr<can_data> data)
  {
    (void) name;
    (void) data;
  }

private:
  std::shared_ptr<EVM::Protocol<can_data>> ptr_can_protocol_;
  ModelSwitch ms_;
  const std::map<PositionSkin, std::string> control_dog_action_map = {
    {PositionSkin::PS_BODYM, "body_middle"},
    {PositionSkin::PS_LBLEG, "left_back_leg"},
    {PositionSkin::PS_BODYL, "body_left"},
    {PositionSkin::PS_LFLEG, "left_front_leg"},
    {PositionSkin::PS_FRONT, "front_chest"},
    {PositionSkin::PS_RFLEG, "right_front_leg"},
    {PositionSkin::PS_BODYR, "body_right"},
    {PositionSkin::PS_RBLEG, "right_back_leg"}
  };
};  // class ElecSkin
}  // namespace motion
}  // namespace cyberdog

#endif  // ELEC_SKIN__ELEC_SKIN_HPP_
