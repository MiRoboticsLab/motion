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

#include <string>
#include <memory>
#include "motion_bridge/file_bridge.hpp"

namespace cyberdog
{
namespace motion
{
FileBridge::FileBridge(const rclcpp::Node::SharedPtr node)
{
  node_ = node;
  lcm_ = std::make_shared<lcm::LCM>(kLCMBirdgeSubscribeURL);
  // std::thread{[this]() {
  //     while (lcm_->good()) {
  //       lcm_->publish(kLCMBridgeElevationChannel, &elevation_);
  //       std::this_thread::sleep_for(std::chrono::milliseconds(50));
  //     }
  //   }
  // }.detach();
  std::ifstream file("/home/harvey/Downloads/user_gait_01.toml");
  std::string s;
  while(getline(file, s)) {
    INFO("%s", s.c_str());
    file_.data += s + "\n";
  }
  lcm_->publish(kLCMBridgeFileChannel, &file_);

}

void FileBridge::Spin()
{
}



void FileBridge::HandleFileBridgeCallback()
{
  // std::ifstream file("");
  // std::string s;
  // while(file >> s) {
  //   INFO("%s", s.c_str());
  // }
}
}  // namespace motion
}  // namespace cyberdog

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node(new rclcpp::Node("elevation_bridge"));
  cyberdog::motion::FileBridge fb(node);
  return 0;
}
