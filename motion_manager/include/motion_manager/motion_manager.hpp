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
#ifndef MOTION_MANAGER__MOTION_HANDLER_HPP_
#define MOTION_MANAGER__MOTION_HANDLER_HPP_
#include <functional>
#include <mutex>
#include <thread>
#include "pluginlib/class_loader.hpp"
#include "bcmgps_base/bcmgps_base.hpp"
#include "protocol/msg/gps_payload.hpp"
#include "manager_base/manager_base.hpp"

namespace cyberdog
{
namespace motion
{
class MotionManager final : public manager::ManagerBase
{
public:
  MotionManager(const std::string & name);
  ~MotionManager();

  void Config() override;
  bool Init() override;
  void Run() override;
  bool SelfCheck() override;
  

public:
  void OnError() override;
  void OnLowPower() override;
  void OnSuspend() override;
  void OnProtected() override;
  void OnActive() override;

private:
  bool IsStateValid();

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};

private:

};  // class MotionManager
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_HANDLER_HPP_
