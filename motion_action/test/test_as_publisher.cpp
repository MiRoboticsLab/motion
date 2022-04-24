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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <motion_action/motion_action.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>
#define GET_TOML_VALUE(a, b, c) \
  if (!cyberdog::common::CyberdogToml::Get(a, b, c)) { \
    FATAL("Cannot get value %s", b); \
    exit(-1); \
  } \

class SimMotionPublisher
{
public:
  SimMotionPublisher(const std::string & name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name);
    motion_cmd_pub_ = node_ptr_->create_publisher<protocol::msg::MotionServoCmd>("motion_servo_cmd", rclcpp::SystemDefaultsQoS());
  }

  void Run(char ** argv)
  {
    cyberdog::motion::MotionServoCmdMsg::SharedPtr msg(new cyberdog::motion::MotionServoCmdMsg);
    std::string cmd_preset = ament_index_cpp::get_package_share_directory("motion_action") + "/preset/" + argv[1] + ".toml";
    toml::value value;
    if (!cyberdog::common::CyberdogToml::ParseFile(cmd_preset, value)) {
      FATAL("Cannot parse %s", cmd_preset.c_str());
      exit(-1);
    }
    GET_TOML_VALUE(value, "motion_id", msg->motion_id);
    GET_TOML_VALUE(value, "cmd_type", msg->cmd_type);
    GET_TOML_VALUE(value, "vel_des", msg->vel_des);
    GET_TOML_VALUE(value, "rpy_des", msg->rpy_des);
    GET_TOML_VALUE(value, "pos_des", msg->pos_des);
    GET_TOML_VALUE(value, "acc_des", msg->acc_des);
    GET_TOML_VALUE(value, "ctrl_point", msg->ctrl_point);
    GET_TOML_VALUE(value, "foot_pose", msg->foot_pose);
    GET_TOML_VALUE(value, "step_height", msg->step_height);
    // HandleTestCmd(msg);
    motion_cmd_pub_->publish(*msg);
    INFO(
      "MotionPublisher publish cmd:\n motion_id: %d\n cmd_type: %d\n vel_des: [%.2f, %.2f, %.2f]\n rpy_des: [%.2f, %.2f, %.2f]\n pos_des: [%.2f, %.2f, %.2f]\n acc_des: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n ctrl_point: [%.2f, %.2f, %.2f]\n foot_pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n step_height: [%.2f, %.2f]\n", msg->motion_id, msg->cmd_type,
      msg->vel_des[0], msg->vel_des[1], msg->vel_des[2], msg->rpy_des[0], msg->rpy_des[1],
      msg->rpy_des[2], msg->pos_des[0], msg->pos_des[1], msg->pos_des[2], msg->acc_des[0],
      msg->acc_des[1], msg->acc_des[2], msg->acc_des[3], msg->acc_des[4], msg->acc_des[5],
      msg->ctrl_point[0], msg->ctrl_point[1], msg->ctrl_point[2], msg->foot_pose[0],
      msg->foot_pose[1], msg->foot_pose[2], msg->foot_pose[3], msg->foot_pose[4],
      msg->foot_pose[5], msg->step_height[0], msg->step_height[1]);
  }

  void Spin()
  {
    // executor_->spin_once();
    rclcpp::shutdown();
  }

private:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr motion_cmd_pub_{nullptr};
  LOGGER_MINOR_INSTANCE("SimMotionPublisher");
};

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("test_as_publisher")
  if(argc < 2){
    FATAL("argc less than 2");
    exit(-1);
  }
  rclcpp::init(argc, argv);
  SimMotionPublisher smm("test_as_publisher");
  smm.Run(argv);
  smm.Spin();
}
