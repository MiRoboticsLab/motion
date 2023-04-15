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
#include "cyberdog_common/cyberdog_log.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "skin_manager/skin_manager.hpp"

namespace cyberdog
{
namespace motion
{

SkinManagerNode:: SkinManagerNode(string::string name)
: Node(name)
{
elec_skin_ = std::make_shared<ElecSkin>();
launch_service_ = this->create_service<std_srvs::srv::SetBool>(
  "enable_elec_skin",
   std::bind(&SkinManagerNode::StartSkinCallback, this,
             std::placeholders::_1, std::placeholders::_2));
skin_mode_service_ = this->create_service<protocol::srv::ElecSkin>(
  "set_elec_skin",
   std::bind(&SkinManagerNode::SetModeCallback,this,
             std::placeholders::_1, std::placeholders::_2));
std::string elec_skin_config = ament_index_cpp::get_package_share_directory("motion_action") +
    "/preset/" + "elec_skin.toml";
  toml::value elec_skin_value;
  if (!cyberdog::common::CyberdogToml::ParseFile(elec_skin_config, elec_skin_value)) {
    FATAL("Cannot parse %s", elec_skin_config.c_str());
    return false;
  }
  // if (!motion_ids.is_table()) {
  //   FATAL("Toml format error");
  //   exit(-1);
  // }
  // toml::value values;
  int8_t default_color = 0;
  int8_t start_direction = 0;
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "default_color", default_color);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "start_direction", start_direction);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "gradual_duration", gradual_duration_);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "stand_gradual_duration_", stand_gradual_duration_);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "twink_gradual_duration_", twink_gradual_duration_);
  cyberdog::common::CyberdogToml::Get(elec_skin_value, "random_gradual_duration_", random_gradual_duration_);
  
  INFO("Default color: %d", default_color);
  INFO("Start direction: %d", start_direction);
  INFO("Gradual duration: %d", gradual_duration_);
  INFO("StandGradual duration: %d", stand_gradual_duration_);
  INFO("TwinkGradual duration: %d", twink_gradual_duration_);
  INFO("RandomGradual duration: %d", random_gradual_duration_);
  if (default_color == 0) {
    change_dir_.push_back(PositionColorChangeDirection::PCCD_WTOB);
    change_dir_.push_back(PositionColorChangeDirection::PCCD_BTOW);
  } else {
    change_dir_.push_back(PositionColorChangeDirection::PCCD_BTOW);
    change_dir_.push_back(PositionColorChangeDirection::PCCD_WTOB);
  }
  if (start_direction == 0) {
    start_dir_ = PositionColorStartDirection::PCSD_FRONT;
  } else {
    start_dir_ = PositionColorStartDirection::PCSD_BACK;
  }
  leg_map.emplace(0, std::vector<PositionSkin>{PositionSkin::PS_RFLEG, PositionSkin::PS_FRONT}); 
  leg_map.emplace(1, std::vector<PositionSkin>{PositionSkin::PS_LFLEG, PositionSkin::PS_BODYL}); 
  leg_map.emplace(2, std::vector<PositionSkin>{PositionSkin::PS_RBLEG, PositionSkin::PS_BODYR}); 
  leg_map.emplace(3, std::vector<PositionSkin>{PositionSkin::PS_LBLEG, PositionSkin::PS_BODYM}); 
}

void StartSkinCallback(
    const std::std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srv::srv::SetBool::Response> response)
{
  if (request->data) {
    INFO("Request to start elec_skin");
    enable_ = true;
  } else {
    INFO("Request to end elsc_skin");
    enable_ = false;
  }
  response->success = true;
}

void SetModeCallback(
    const std::shared_ptr<protocol::srv::ElecSkin::Request> request,
    const std::shared_ptr<protocol::srv::ElecSkin::Response> response)
{
  if(!enable_) {
    return;
  }
  switch(request->mode) {
    case 0:
      black_skin_ = true;
      break;

    case 1:
      white_skin_ = true;
      break;

    case 2:
      front_skin_ = true;
      break;
      
    case 3:
      back_skin_ = true;
      break;

    case 4:
      flash_skin_ = true;
      break;

    case 5:
      random_skin_ = true;
      break;

    case 6:
      move_skin_ = true;
      break;
      
    default:
      break;
  }
}

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc,argv);
//   rclcpp::executors::MultiThreadedExecutor executor;
//   auto node = std::make_shared<cyberdog::motion::SkinManagerNode>("skin_manager");
//   executor.add_node(node);
//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }
}
}