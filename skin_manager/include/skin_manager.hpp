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
  
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr launch_service_;
  rclcpp::Service<protocol::srv::ElecSkin>::SharedPtr skin_mode_service_;
  std::unordered_map<uint8_t, std::vector<PositionSkin>> leg_map;
  std::vector<PositionColorChangeDirection> change_dir_;
  PositionColorStartDirection start_dir_;
  int32_t gradual_duration_{0};
  bool enable_{false};
  bool align_contact_{false};
};
}
}


#endif // SKIN_MANAGER__SKIN_MANAGER_HPP_