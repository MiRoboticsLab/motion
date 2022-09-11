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
#ifndef MOTION_BRIDGE__FILE_BRIDGE_HPP_
#define MOTION_BRIDGE__FILE_BRIDGE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <lcm/lcm-cpp.hpp>
#include "protocol/lcm/file_send_lcmt.hpp"
#include <protocol/lcm/file_recv_lcmt.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <fstream>
#include <string>
#include <memory>
#include "motion_action/motion_macros.hpp"

namespace cyberdog
{
namespace motion
{
class FileBridge
{
public:
  explicit FileBridge(const rclcpp::Node::SharedPtr node);
  ~FileBridge() {}
  void Spin();

private:
  void HandleFileBridgeCallback();
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<lcm::LCM> lcm_;
  file_send_lcmt file_;
};  // class FileBridge
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_BRIDGE__FILE_BRIDGE_HPP_
