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
#include <vector>
#include <map>
#include "elec_skin/elec_skin_base.hpp"
#include "elec_skin/elec_skin.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "std_msgs/msg/string.hpp"

#define   NODE_NAME   "elec_skin_node_test"

// 使用字符分割
void string_split(
  const std::string & str,
  const char split, std::vector<std::string> & res)
{
  if (str == "") {
    return;
  }
  // 在字符串末尾也加入分隔符，方便截取最后一段
  std::string strs = str + split;
  size_t pos = strs.find(split);
  // 若找不到内容则字符串搜索函数返回 npos
  while (pos != strs.npos) {
    std::string temp = strs.substr(0, pos);
    res.push_back(temp);
    // 去掉已分割的字符串,在剩下的字符串中进行分割
    strs = strs.substr(pos + 1, strs.size());
    pos = strs.find(split);
  }
}

class ElecSkinNode : public rclcpp::Node
{
public:
  ElecSkinNode()
  : Node(NODE_NAME)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "elec_skin_test",
      10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::vector<std::string> str_vec;
        string_split(msg->data, ',', str_vec);
        if (str_vec.size() < 1) {
          return;
        }
        uint8_t method = 255;
        std::map<std::string, uint8_t> m_cmd {{"model", 0}, {"position", 1}};
        if (m_cmd.find(str_vec[0]) == m_cmd.end()) {
          return;
        }
        INFO("handle elec skin control:>>>");
        method = m_cmd[str_vec[0]];
        switch (method) {
          case 0:
            {
              if (str_vec.size() != 3) {
                INFO("parameter counts error, should be 3!");
                return;
              }
              cyberdog::motion::ModelSwitch ms =
              cyberdog::motion::ModelSwitch(atoi(str_vec[1].c_str()));
              uint16_t milsecs = static_cast<uint16_t>(atoi(str_vec[2].c_str()));
              can_control_->ModelControl(ms, milsecs);
            }
            break;
          case 1:
            {
              if (str_vec.size() != 5) {
                INFO("parameter counts error, should be 4!");
                return;
              }
              cyberdog::motion::PositionSkin ps =
              cyberdog::motion::PositionSkin(atoi(str_vec[1].c_str()));
              cyberdog::motion::PositionColorChangeDirection pccd =
              cyberdog::motion::PositionColorChangeDirection(atoi(str_vec[2].c_str()));
              cyberdog::motion::PositionColorStartDirection pcsd =
              cyberdog::motion::PositionColorStartDirection(atoi(str_vec[3].c_str()));
              uint16_t milsecs = static_cast<uint16_t>(atoi(str_vec[4].c_str()));
              can_control_->PositionContril(ps, pccd, pcsd, milsecs);
            }
            break;
          default:
            break;
        }
      });
    can_control_ = std::make_unique<cyberdog::motion::ElecSkin>();
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unique_ptr<cyberdog::motion::ElecSkinBase> can_control_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("elec_skin");
  rclcpp::spin(std::make_shared<ElecSkinNode>());
  rclcpp::shutdown();
  return 0;
}
