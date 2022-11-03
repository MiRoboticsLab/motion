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
#ifndef ELEC_SKIN__ELEC_SKIN_BASE_HPP_
#define ELEC_SKIN__ELEC_SKIN_BASE_HPP_

#include <stdint.h>

namespace cyberdog
{
namespace motion
{

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

class ElecSkinBase
{
public:
  virtual void ModelControl(ModelSwitch ms, uint16_t milsecs) = 0;
  virtual void PositionContril(
    PositionSkin ps, PositionColorChangeDirection pccd,
    PositionColorStartDirection pcsd, uint16_t milsecs) = 0;
};

}  // namespace motion
}  // namespace cyberdog

#endif  // ELEC_SKIN__ELEC_SKIN_BASE_HPP_
