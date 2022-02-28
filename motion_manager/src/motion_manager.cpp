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
#include "rclcpp/rclcpp.hpp"
#include "motion_manager/motion_manager.hpp"

cyberdog::motion::MotionManager::MotionManager(const std::string & name)
  : manager::ManagerBase(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
}

cyberdog::motion::MotionManager::~MotionManager()
{}

void cyberdog::motion::MotionManager::Config()
{
  // TODO: get info from configure
}

bool cyberdog::motion::MotionManager::Init()
{
  // TODO: register manager base functions
  return true;
}

void cyberdog::motion::MotionManager::Run()
{
  rclcpp::spin(node_ptr_);
  rclcpp::shutdown();
}

bool cyberdog::motion::MotionManager::SelfCheck()
{
  // TODO: check all motions from config
  return true;
}

bool cyberdog::motion::MotionManager::IsStateValid()
{
  // TODO: check state from behavior tree
  return true;
}

void cyberdog::motion::MotionManager::OnError()
{
  std::cout << "on error\n";
}

void cyberdog::motion::MotionManager::OnLowPower()
{
  std::cout << "on lowpower\n";
}

void cyberdog::motion::MotionManager::OnSuspend()
{
  std::cout << "on suspend\n";
}

void cyberdog::motion::MotionManager::OnProtected()
{
  std::cout << "on protect\n";
}

void cyberdog::motion::MotionManager::OnActive()
{
  std::cout << "on active\n";
}
