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
#include <std_msgs/msg/int16.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cyberdog_debug/backtrace.hpp>
#include <motion_action/motion_action.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>


class SimMotionManager
{
public:
  SimMotionManager(const std::string & name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name);
    node_ptr_->declare_parameter("publish_url", cyberdog::motion::PUBLISH_URL);
    node_ptr_->get_parameter("publish_url", publish_url_);
    node_ptr_->declare_parameter("subscribe_url", cyberdog::motion::SUBSCRIBE_URL);
    node_ptr_->get_parameter("subscribe_url", subscribe_url);
    node_ptr_->declare_parameter<std::string>("cmd_preset");
    node_ptr_->get_parameter("cmd_preset", cmd_preset_);
    if (cmd_preset_.empty()) {
      FATAL("Preset cmd cannot be empty");
      exit(-1);
    }
    msg_t_.reset(new protocol::msg::MotionServoCmd);
    res_t_.reset(new protocol::msg::MotionStatus);
    res_t_->motor_error.resize(12);
    last_msg_.reset(new protocol::msg::MotionServoCmd);
    srv_req_.reset(new protocol::srv::MotionResultCmd::Request);
    executor_.reset(new rclcpp::executors::SingleThreadedExecutor);
    executor_->add_node(node_ptr_);
    ma_.reset(new cyberdog::motion::MotionAction);
    ma_->Init(publish_url_, subscribe_url);
    ma_->RegisterFeedback(
      std::bind(
        &SimMotionManager::FeedbackCallback, this,
        std::placeholders::_1));
    motion_cmd_sub_ = node_ptr_->create_subscription<protocol::msg::MotionServoCmd>(
      "motion_servo_cmd",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&SimMotionManager::HandleTestServoCmd, this, std::placeholders::_1));
    motion_result_queue_sub_ = node_ptr_->create_subscription<std_msgs::msg::Int16>(
      "motion_result_queue",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&SimMotionManager::HandleTestResultQueueCmd, this, std::placeholders::_1));
    motion_result_srv_ =
      node_ptr_->create_service<protocol::srv::MotionResultCmd>(
      "motion_result_cmd",
      std::bind(
        &SimMotionManager::HandleTestResultCmd, this, std::placeholders::_1,
        std::placeholders::_2));
  }

  void HandleTestResultCmd(const protocol::srv::MotionResultCmd_Request::SharedPtr req,
                          const protocol::srv::MotionResultCmd_Response::SharedPtr)
  {
    // if(*last_msg_ == *msg)
      // return;
    srv_req_->motion_id = req->motion_id; 
    srv_req_->duration = req->duration;
    srv_req_->step_height.resize(2);
    GET_VALUE(req->step_height, srv_req_->step_height, 2, "step_height");
    srv_req_->vel_des.resize(3);
    GET_VALUE(req->vel_des, srv_req_->vel_des, 3, "vel_des");
    srv_req_->rpy_des.resize(3);
    GET_VALUE(req->rpy_des, srv_req_->rpy_des, 3, "rpy_des");
    srv_req_->pos_des.resize(3);
    GET_VALUE(req->pos_des, srv_req_->pos_des, 3, "pos_des");
    srv_req_->ctrl_point.resize(3);
    GET_VALUE(req->ctrl_point, srv_req_->ctrl_point, 3, "ctrl_point");
    srv_req_->acc_des.resize(6);
    GET_VALUE(req->acc_des, srv_req_->acc_des, 6, "acc_des");
    srv_req_->foot_pose.resize(6);
    GET_VALUE(req->foot_pose, srv_req_->foot_pose, 6, "foot_pose");

    ma_->Execute((srv_req_));
    INFO(
      "MotionManager send ResultCmd:\n motion_id: %d\n vel_des: [%.2f, %.2f, %.2f]\n rpy_des: [%.2f, %.2f, %.2f]\n pos_des: [%.2f, %.2f, %.2f]\n acc_des: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n ctrl_point: [%.2f, %.2f, %.2f]\n foot_pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n step_height: [%.2f, %.2f]\n", srv_req_->motion_id,
      srv_req_->vel_des[0], srv_req_->vel_des[1], srv_req_->vel_des[2], srv_req_->rpy_des[0], srv_req_->rpy_des[1],
      srv_req_->rpy_des[2], srv_req_->pos_des[0], srv_req_->pos_des[1], srv_req_->pos_des[2], srv_req_->acc_des[0],
      srv_req_->acc_des[1], srv_req_->acc_des[2], srv_req_->acc_des[3], srv_req_->acc_des[4], srv_req_->acc_des[5],
      srv_req_->ctrl_point[0], srv_req_->ctrl_point[1], srv_req_->ctrl_point[2], srv_req_->foot_pose[0],
      srv_req_->foot_pose[1], srv_req_->foot_pose[2], srv_req_->foot_pose[3], srv_req_->foot_pose[4],
      srv_req_->foot_pose[5], srv_req_->step_height[0], srv_req_->step_height[1]);
  }


  void HandleTestServoCmd(const protocol::msg::MotionServoCmd::SharedPtr msg)
  {
    // if(*last_msg_ == *msg)
      // return;
    last_msg_ = msg;
    msg_t_->motion_id = msg->motion_id;
    msg_t_->cmd_type = msg->cmd_type;
    msg_t_->step_height.resize(2);
    GET_VALUE(msg->step_height, msg_t_->step_height, 2, "step_height");
    msg_t_->vel_des.resize(3);
    GET_VALUE(msg->vel_des, msg_t_->vel_des, 3, "vel_des");
    msg_t_->rpy_des.resize(3);
    GET_VALUE(msg->rpy_des, msg_t_->rpy_des, 3, "rpy_des");
    msg_t_->pos_des.resize(3);
    GET_VALUE(msg->pos_des, msg_t_->pos_des, 3, "pos_des");
    msg_t_->ctrl_point.resize(3);
    GET_VALUE(msg->ctrl_point, msg_t_->ctrl_point, 3, "ctrl_point");
    msg_t_->acc_des.resize(6);
    GET_VALUE(msg->acc_des, msg_t_->acc_des, 6, "acc_des");
    msg_t_->foot_pose.resize(6);
    GET_VALUE(msg->foot_pose, msg_t_->foot_pose, 6, "foot_pose");

    ma_->Execute((msg_t_));
    INFO(
      "MotionManager send ServoCmd:\n motion_id: %d\n cmd_type: %d\n vel_des: [%.2f, %.2f, %.2f]\n rpy_des: [%.2f, %.2f, %.2f]\n pos_des: [%.2f, %.2f, %.2f]\n acc_des: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n ctrl_point: [%.2f, %.2f, %.2f]\n foot_pose: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n step_height: [%.2f, %.2f]\n", msg_t_->motion_id, msg_t_->cmd_type,
      msg_t_->vel_des[0], msg_t_->vel_des[1], msg_t_->vel_des[2], msg_t_->rpy_des[0], msg_t_->rpy_des[1],
      msg_t_->rpy_des[2], msg_t_->pos_des[0], msg_t_->pos_des[1], msg_t_->pos_des[2], msg_t_->acc_des[0],
      msg_t_->acc_des[1], msg_t_->acc_des[2], msg_t_->acc_des[3], msg_t_->acc_des[4], msg_t_->acc_des[5],
      msg_t_->ctrl_point[0], msg_t_->ctrl_point[1], msg_t_->ctrl_point[2], msg_t_->foot_pose[0],
      msg_t_->foot_pose[1], msg_t_->foot_pose[2], msg_t_->foot_pose[3], msg_t_->foot_pose[4],
      msg_t_->foot_pose[5], msg_t_->step_height[0], msg_t_->step_height[1]);
  }

  void HandleTestResultQueueCmd(const std_msgs::msg::Int16::SharedPtr msg)
  {
    std::string cmd_preset = ament_index_cpp::get_package_share_directory("motion_action") + "/preset/" + std::to_string(msg->data) + ".toml";
    toml::value steps;
    if (!cyberdog::common::CyberdogToml::ParseFile(cmd_preset, steps)) {
      FATAL("Cannot parse %s", cmd_preset.c_str());
      exit(-1);
    }
    if(!steps.is_table()) {
      FATAL("Toml format error");
      exit(-1);
    }
    toml::value values;
    cyberdog::common::CyberdogToml::Get(steps, "step", values);
    for(size_t i = 0; i < values.size(); i++) {
      auto value = values.at(i);
      robot_control_cmd_lcmt lcm_base;
      GET_TOML_VALUE(value, "mode", lcm_base.mode);
      GET_TOML_VALUE(value, "gait_id", lcm_base.gait_id);
      GET_TOML_VALUE(value, "contact", lcm_base.contact);
      GET_TOML_VALUE(value, "life_count", lcm_base.life_count);
      GET_TOML_VALUE(value, "value", lcm_base.value);
      GET_TOML_VALUE(value, "duration", lcm_base.duration);
      std::vector<float> tmp;
      GET_TOML_VALUE_ARR(value, "vel_des", tmp, lcm_base.vel_des);
      GET_TOML_VALUE_ARR(value, "rpy_des", tmp, lcm_base.rpy_des);
      GET_TOML_VALUE_ARR(value, "pos_des", tmp, lcm_base.pos_des);
      GET_TOML_VALUE_ARR(value, "acc_des", tmp, lcm_base.acc_des);
      GET_TOML_VALUE_ARR(value, "ctrl_point", tmp, lcm_base.ctrl_point);
      GET_TOML_VALUE_ARR(value, "foot_pose", tmp, lcm_base.foot_pose);
      GET_TOML_VALUE_ARR(value, "step_height", tmp, lcm_base.step_height);
      ma_->Execute(lcm_base);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void Run()
  {
    protocol::msg::MotionServoCmd::SharedPtr msg(new protocol::msg::MotionServoCmd);
    toml::value value;
    if (!cyberdog::common::CyberdogToml::ParseFile(cmd_preset_, value)) {
      FATAL("Cannot parse %s", cmd_preset_.c_str());
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
    HandleTestServoCmd(msg);
  }

  void Spin()
  {
    executor_->spin();
    rclcpp::shutdown();
  }

private:
  bool CompareMotionServoResponse(protocol::msg::MotionStatus::SharedPtr res1, protocol::msg::MotionStatus::SharedPtr res2)
  {
    bool flag = true;
    for (uint8_t i = 0; i < 12; ++i) {
      flag &= (res1->motor_error[i] == res2->motor_error[i]);
    }
    return (res1->motion_id == res2->motion_id &&
            res1->contact == res2->contact &&
            res1->order_process_bar == res2->order_process_bar &&
            res1->switch_status == res2->switch_status &&
            res1->ori_error == res2->ori_error &&
            res1->footpos_error == res2->footpos_error &&
            flag );

  }
  void FeedbackCallback(protocol::msg::MotionStatus::SharedPtr msg)
  {
    if(CompareMotionServoResponse(res_t_, msg))
      return;
    res_t_ = msg;
    INFO(
      "MotionManager get res:\n motion_id: %d\n contact: %d\n order_process_bar: %d\n switch_status: %d\n ori_error: %d\n footpos_error: %d\n motor_error: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n", msg->motion_id, msg->contact, msg->order_process_bar, msg->switch_status, msg->ori_error, msg->footpos_error,
            msg->motor_error[0], msg->motor_error[1], msg->motor_error[2], msg->motor_error[3],
            msg->motor_error[4], msg->motor_error[5], msg->motor_error[6], msg->motor_error[7],
            msg->motor_error[8], msg->motor_error[9], msg->motor_error[10], msg->motor_error[11]);

  }
  std::shared_ptr<cyberdog::motion::MotionAction> ma_;
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_{nullptr};
  rclcpp::Subscription<protocol::msg::MotionServoCmd>::SharedPtr motion_cmd_sub_{nullptr};
  rclcpp::Service<protocol::srv::MotionResultCmd>::SharedPtr motion_result_srv_{nullptr};
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr motion_result_queue_sub_{nullptr}; 
  protocol::msg::MotionServoCmd::SharedPtr msg_t_{nullptr}, last_msg_{nullptr};
  protocol::srv::MotionResultCmd::Request::SharedPtr srv_req_{nullptr};
  protocol::srv::MotionResultCmd::Response::SharedPtr srv_res_{nullptr};
  protocol::msg::MotionStatus::SharedPtr res_t_;
  std::string publish_url_, subscribe_url, cmd_preset_;
  LOGGER_MINOR_INSTANCE("SimMotionManager");
};

int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE("test_as_manager")
  rclcpp::init(argc, argv);
  cyberdog::debug::register_signal();
  SimMotionManager smm("test_as_manager");
  smm.Run();
  smm.Spin();
}
