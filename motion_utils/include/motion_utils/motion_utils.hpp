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
#ifndef MOTION_UTILS__MOTION_UTILS_HPP_
#define MOTION_UTILS__MOTION_UTILS_HPP_
#include <unistd.h>
#include <lcm/lcm-cpp.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <map>
#include <vector>
#include <mutex>
#include "motion_action/motion_macros.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/lcm/robot_control_response_lcmt.hpp"
#include "protocol/lcm/robot_control_cmd_lcmt.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace cyberdog
{
namespace motion
{

class OdomHelper
{
public:
  explicit OdomHelper(rclcpp::Node::SharedPtr node)
  {
    node_ = node;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      kBridgeOdomTopicName,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&OdomHelper::HandleOdomCallback, this, std::placeholders::_1));
    odom_.reset(new nav_msgs::msg::Odometry);
  }
  ~OdomHelper() {}
  double GetDistance()
  {
    return distance_;
  }
  bool SetStartPoint()
  {
    std::unique_lock<std::mutex> lk(odom_msg_mutex_);
    odom_waiting_ = true;
    if (cv_.wait_for(lk, std::chrono::milliseconds(1000)) == std::cv_status::timeout) {
      ERROR("Wait for odom timeout");
      odom_waiting_ = false;
      return false;
    }
    odom_waiting_ = false;
    last_x_ = odom_->pose.pose.position.x;
    last_y_ = odom_->pose.pose.position.y;
    distance_ = 0;
    start_point_set_ = true;
    return true;
  }
  void Reset()
  {
    start_point_set_ = false;
    odom_waiting_ = false;
  }

private:
  void HandleOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (odom_waiting_) {cv_.notify_one();}
    odom_ = msg;
    if (!start_point_set_) {return;}
    auto delta_x = odom_->pose.pose.position.x - last_x_;
    auto delta_y = odom_->pose.pose.position.y - last_y_;
    distance_ += sqrt(delta_x * delta_x + delta_y * delta_y);
    last_x_ = odom_->pose.pose.position.x;
    last_y_ = odom_->pose.pose.position.y;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  std::mutex odom_msg_mutex_;
  std::condition_variable cv_;
  double distance_{0};
  double last_x_{0}, last_y_{0};
  bool start_point_set_{false}, odom_waiting_{false};
};  // class OdomHelper

class MotionUtils final
{
public:
  static MotionUtils & GetMotionUtils()
  {
    static MotionUtils mu_;
    return mu_;
  }
  ~MotionUtils();

public:
  /**
   * @brief 控制机器人按照特定的时间和完整的伺服指令行走
   *
   * @param duration 行走时间, 单位ms
   * @param msg 伺服指令
   * @return true: 无异常, false: 异常退出
   */
  bool ExecuteWalkDuration(int duration, const MotionServoCmdMsg::SharedPtr msg);
  /**
   * @brief 控制机器人按照特定的时间和速度, 以自变频步态(motion_id: 303)行走
   *
   * @param duration 行走时间, 单位ms
   * @param vel_x x方向的速度, 单位m/s
   * @param vel_y y方向的速度, 单位m/s
   * @param omega 角速度, 单位rad/s
   * @return true: 无异常, false: 异常退出
   */
  bool ExecuteWalkDuration(int duration, float vel_x = 0.0, float vel_y = 0.0, float omega = 0.0);
  /**
   * @brief 控制机器人按照完整的伺服指令行走特定的距离
   *
   * @param distance 行走距离, 单位m
   * @param msg 伺服指令
   * @return true: 无异常, false: 异常退出
   */
  bool ExecuteWalkDistance(double distance, MotionServoCmdMsg::SharedPtr msg);
  /**
   * @brief 控制机器人按照特定的速度, 以自变频步态(motion_id: 303)行走特定的距离
   *
   * @param distance 行走距离, 单位m
   * @param vel_x x方向的速度, 单位m/s
   * @param vel_y y方向的速度, 单位m/s
   * @param omega 角速度，单位rad/s
   * @return true: 无异常, false: 异常退出
   */
  bool ExecuteWalkDistance(
    double distance, float vel_x = 0.0, float vel_y = 0.0,
    float omega = 0.0);

private:
  MotionUtils();
  MotionUtils(const MotionUtils &) = delete;
  MotionUtils & operator=(const MotionUtils &) = delete;
  void HandleMotionStatusCallback(const MotionStatusMsg::SharedPtr msg)
  {
    motion_status_ = msg;
    if (motion_status_waiting_) {motion_status_cv_.notify_one();}
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<MotionServoCmdMsg>::SharedPtr servo_cmd_pub_;
  rclcpp::Subscription<MotionStatusMsg>::SharedPtr motion_status_sub_;
  rclcpp::Client<MotionResultSrv>::SharedPtr result_cmd_client_;
  MotionStatusMsg::SharedPtr motion_status_;
  std::shared_ptr<OdomHelper> odom_helper_;
  std::mutex motion_status_mutex_;
  std::condition_variable motion_status_cv_;
  bool motion_status_waiting_{false};
  LOGGER_MINOR_INSTANCE("MotionUtils");
};  // class MotionUtils
}  // namespace motion
}  // namespace cyberdog
#endif  // MOTION_UTILS__MOTION_UTILS_HPP_
