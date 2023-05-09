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

#include <memory>
#include <string>
#include <vector>
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "skin_manager/skin_manager.hpp"
#include "elec_skin/elec_skin.hpp"

namespace cyberdog
{
namespace motion
{

SkinManagerNode::SkinManagerNode(std::string name)
: Node(name)
{ 
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(this->get_node_base_interface());
  elec_skin_ = std::make_shared<ElecSkin>();
  launch_service_ = this->create_service<std_srvs::srv::SetBool>(
    "enable_elec_skin",
    std::bind(
      &SkinManagerNode::StartSkinCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  skin_mode_service_ = this->create_service<protocol::srv::ElecSkin>(
    "set_elec_skin",
    std::bind(
      &SkinManagerNode::SetModeCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  std::string elec_skin_config = ament_index_cpp::get_package_share_directory("skin_manager") +
    "/config/" + "elec_skin.toml";
  toml::value elec_skin_value;
  if (!cyberdog::common::CyberdogToml::ParseFile(elec_skin_config, elec_skin_value)) {
    FATAL("Cannot parse %s", elec_skin_config.c_str());
    // return false;
  }
  // if (!motion_ids.is_table()) {
  //   FATAL("Toml format error");
  //   exit(-1);
  // }
  // toml::value values;
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "default_color", default_color_);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "start_direction", start_direction_);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "gradual_duration", gradual_duration_);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "defaul_duration", defaul_duration_);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "enable", enable_);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "align_contact", align_contact_);
  INFO("Default color: %d", default_color_);
  INFO("Start direction: %d", start_direction_);
  INFO("Gradual duration: %d", gradual_duration_);
  INFO("defaul_duration: %d", defaul_duration_);
  INFO("enable: %d", enable_);
  if (default_color_ == 0) {
    change_dir_.push_back(PositionColorChangeDirection::PCCD_WTOB);
    change_dir_.push_back(PositionColorChangeDirection::PCCD_BTOW);
  } else {
    change_dir_.push_back(PositionColorChangeDirection::PCCD_BTOW);
    change_dir_.push_back(PositionColorChangeDirection::PCCD_WTOB);
  }
  if (start_direction_ == 0) {
    start_dir_ = PositionColorStartDirection::PCSD_FRONT;
  } else {
    start_dir_ = PositionColorStartDirection::PCSD_BACK;
  }
  liftdown_color_ = PositionColorChangeDirection::PCCD_BTOW;
  liftup_color_ = PositionColorChangeDirection::PCCD_WTOB;
  leg_map.emplace(0, std::vector<PositionSkin>{PositionSkin::PS_RFLEG, PositionSkin::PS_FRONT});
  leg_map.emplace(1, std::vector<PositionSkin>{PositionSkin::PS_LFLEG, PositionSkin::PS_BODYL});
  leg_map.emplace(2, std::vector<PositionSkin>{PositionSkin::PS_RBLEG, PositionSkin::PS_BODYR});
  leg_map.emplace(3, std::vector<PositionSkin>{PositionSkin::PS_LBLEG, PositionSkin::PS_BODYM});
  std::thread{[this] {this->executor_->spin();}}.detach();
}

void SkinManagerNode::StartSkinCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    INFO("Request to start elec_skin");
    enable_ = true;
    WriteTomlFile();
  } else {
    INFO("Request to stop elsc_skin");
    enable_ = false;
    ShowDefaultSkin();
    SetAlignContact(true);
    WriteTomlFile();
  }
  response->success = true;
}

void SkinManagerNode::SetModeCallback(
  const std::shared_ptr<protocol::srv::ElecSkin::Request> request,
  const std::shared_ptr<protocol::srv::ElecSkin::Response> response)
{
  if (!enable_) {
    return;
  }
  switch (request->mode) {
    case 0:
      align_contact_ = false;
      WriteTomlFile();
      INFO("ShowBlackElecSkin");
      ShowBlackElecSkin(request->wave_cycle_time);
      break;

    case 1:
      align_contact_ = false;
      WriteTomlFile();
      INFO("ShowWhiteElecSkin");
      ShowWhiteElecSkin(request->wave_cycle_time);
      break;

    case 2:
      align_contact_ = false;
      WriteTomlFile();
      INFO("ShowFrontElecSkin");
      ShowFrontElecSkin(request->wave_cycle_time);
      break;

    case 3:
      align_contact_ = false;
      WriteTomlFile();
      INFO("ShowBackElecSkin");
      ShowBackElecSkin(request->wave_cycle_time);
      break;

    case 4:
      align_contact_ = false;
      WriteTomlFile();
      INFO("ShowFlashElecSkin");
      ShowFlashElecSkin(request->wave_cycle_time);
      break;

    case 5:
      align_contact_ = false;
      WriteTomlFile();
      INFO("ShowRandomElecSkin");
      ShowRandomElecSkin(request->wave_cycle_time);
      break;

    case 6:
      align_contact_ = true;
      WriteTomlFile();
      INFO("ShowMoveElecSkin");
      if (!request->wave_cycle_time) {
        liftdown_color_ = PositionColorChangeDirection::PCCD_BTOW;
        liftup_color_ = PositionColorChangeDirection::PCCD_WTOB;
      } else {
        liftdown_color_ = PositionColorChangeDirection::PCCD_WTOB;
        liftup_color_ = PositionColorChangeDirection::PCCD_BTOW;
      }
      break;

    default:
      break;
  }
  response->success = true;
}

}  // namespace motion
}  // namespace cyberdog
