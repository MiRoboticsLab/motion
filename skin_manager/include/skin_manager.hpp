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
#ifndef SKIN_MANAGER__SKIN_MANAGER_HPP_
#define SKIN_MANAGER__SKIN_MANAGER_HPP_
#include <unistd.h>
#include <string>
#include <thread>
#include <memory>
#include <map>
#include <vector>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/elec_skin.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "elec_skin/elec_skin_base.hpp"
namespace cyberdog
{
namespace motion
{
class SkinManagerNode : public rclcpp::Node
{
pubilc:
  explicit SkinManagerNode(std::string name);

private:
  void StartSkinCallback(
    const std::std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srv::srv::SetBool::Response> response);
  void SetModeCallback(
    const std::shared_ptr<protocol::srv::ElecSkin::Request> request,
    const std::shared_ptr<protocol::srv::ElecSkin::Response> response);

  void ShowMoveElecSkin(std::vector<uint_8> & msg) {
    static auto last_contact = std::vector<uint8_t>(4, 0);
    auto contact = std::vector<uint8_t>(4, 0);
    for(uint8_t i = 0; i < 4; ++i) {
      contact[i] = msg[i] >0 ? 1: 0;
      if (contact[i] == last_contact[i]) {
        continue;
      }
      last_contact[i] = contact[i];
      if (contact[i] == 1) {
        WARN("Leg %d liftdown", i);
        for (auto p : leg_map[i]) {
          elec_skin_->PositionContril(
            p,
            change_dir_.front(),
            start_dir_,
            gradual_duration_);
        }
      } else {
        WARN("Leg %d liftup", i);
        for (auto p : leg_map[i]) {
          elec_skin_->PositionContril(
            p,
            change_dir_.back(),
            start_dir_,
            gradual_duration_);
        }
      }
    }  
  }

  void ShowBlackElecSkin() {
    if(!align_contact_) {
      grandual_duration_ = request->wave_cycle_time;
      INFO("ShowBlackElecSkin");
      for(uint8_t i=0; i<4; ++i) {
        for(auto p : leg_map[i]) {
          elec_skin_->PositionContril(
            p,
            change_dir_.front(),
            start_dir_,
            grandual_duration_);
        }
      }  
    }
  }

  void ShowWhiteElecSkin() {
    if(!align_contact_) {
      grandual_duration_ = request->wave_cycle_time;
      INFO("ShowWhiteElecSkin");
      for(uint8_t i=0; i<4; i++) {
        for(auto p : leg_map[i]) {
          elec_skin_->PositionContril(
            p,
            change_dir_.back(),
            start_dir_,
            grandual_duration_);
        }
      }
    }
  }
  
  void ShowFrontElecSkin() {
    align_contact_ = false;
    stand_grandual_duration_ = request->wave_cycle_time;
    INFO("ShowFrontElecSkin");
    elec_skin_->ModelControl(ModelSwitch::MS_WAVEF, stand_gradual_duration_);
  }

  void ShowBackElecSkin() {
    align_contact_ = false;
    stand_grandual_duration_ = request->wave_cycle_time;
    INFO("ShowBackElecSkin");
    elec_skin_->ModelControl(ModelSwitch::MS_WAVEB, stand_gradual_duration_);
  }

  void ShowFlashElecSkin() {
    align_contact_ = false;
    twink_gradual_duration_ = request->wave_cycle_time;
    INFO("ShowFlashElecSkin");
    elec_skin_->ModelControl(ModelSwitch::MS_FLASH, twink_gradual_duration_);
  }

  void ShowRandomElecSkin() {
    align_contact_ = false;
    random_gradual_duration_ = request->wave_cycle_time;
    INFO("ShowRandomElecSkin");
    elec_skin_->ModelControl(ModelSwitch::MS_RANDOM, random_gradual_duration_);
  }
  
  void ShowDefaultSkin(bool default_color, bool align_contact)
  {
    INFO("ShowDefaultSkin");
    align_contact_ = align_contact;
    PositionColorChangeDirection dir;
    dir = default_color ? change_dir_.front() : change_dir_.back();
    for (auto leg : leg_map) {
      for (auto p : leg.second) {
        elec_skin_->PositionContril(
          p,
          dir,
          start_dir_,
          gradual_duration_);
      }
    }
  }
  
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr launch_service_;
  rclcpp::Service<protocol::srv::ElecSkin>::SharedPtr skin_mode_service_;
  std::unordered_map<uint8_t, std::vector<PositionSkin>> leg_map;
  std::vector<PositionColorChangeDirection> change_dir_;
  PositionColorStartDirection start_dir_;
  int32_t gradual_duration_{0};
  int32_t stand_gradual_duration_{0};
  int32_t twink_gradual_duration_{0};
  int32_t random_gradual_duration_{0};
  bool enable_{false};
  bool align_contact_{false};
  bool black_skin_{false};
  bool white_skin_{false};
  bool front_skin_{false};
  bool back_skin_{false};
  bool flash_skin_{false};
  bool random_skin_{false};
  bool move_skin_{false};
};
}
}


#endif // SKIN_MANAGER__SKIN_MANAGER_HPP_