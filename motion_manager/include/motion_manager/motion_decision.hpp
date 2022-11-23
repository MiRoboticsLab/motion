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
#ifndef MOTION_MANAGER__MOTION_DECISION_HPP_
#define MOTION_MANAGER__MOTION_DECISION_HPP_
#include <thread>
#include <string>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "motion_action/motion_macros.hpp"
#include "motion_action/motion_action.hpp"
#include "motion_manager/motion_handler.hpp"
#include "cyberdog_common/cyberdog_msg_queue.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace cyberdog
{
namespace motion
{

class LaserHelper final
{
public:
  explicit LaserHelper(const rclcpp::Node::SharedPtr node)
  {
    laser_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      kGlobalScanTopicName,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&LaserHelper::HandleLaserScanCallback, this, std::placeholders::_1));
    laser_pub_ = node->create_publisher<sensor_msgs::msg::LaserScan>("scan_filterd", 1);
    msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
  }
  bool IsStuck()
  {
    int32_t i = 0;
    for (auto range : msg_->ranges) {
      ++i;
      if (range == 0) {
        continue;
      }
      if (range < ranges_threshold_->at(i)) {
        return true;
      }
    }
    return false;
  }

private:
  void HandleLaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    msg_->header = msg->header;
    msg_->intensities = msg->intensities;
    msg_->angle_increment = msg->angle_increment;
    msg_->range_max = msg->range_max;
    msg_->range_min = msg->range_min;
    msg_->scan_time = msg->scan_time;
    msg_->angle_max = msg->angle_max - msg->angle_increment * (msg->ranges.size() - end_);
    msg_->angle_min = msg->angle_min + msg->angle_increment * begin_;
    // msg_->ranges.assign(msg->ranges.begin() + begin_, msg->ranges.begin() + end_);
    // laser_pub_->publish(*msg_);
    static bool threshold_construct_ = false;
    if (!threshold_construct_) {
      ranges_threshold_ = std::make_shared<std::vector<float>>(end_ - begin_, 0.0);
      auto fov = msg_->angle_max - msg_->angle_min;
      for (std::size_t i = 0; i < ranges_threshold_->size(); ++i) {
        ranges_threshold_->at(i) = min_distance_ / cos(fov / 2 - msg_->angle_increment * i);
      }
      threshold_construct_ = true;
    }
    msg_->ranges = *ranges_threshold_;
    laser_pub_->publish(*msg_);
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  sensor_msgs::msg::LaserScan::SharedPtr msg_;
  std::shared_ptr<std::vector<float>> ranges_threshold_;
  int32_t begin_{700}, end_{1320};
  float min_distance_ {0.0};  // 0.22
};

class MotionDecision final
{
public:
  enum class DecisionStatus : int32_t
  {
    kExecutingApp = 0,
    kExecutingAudio = 1,
    kExecutingVis = 2,
    kIdle = 100,
  };  // enum class DecisionStatus
  MotionDecision(const rclcpp::Node::SharedPtr & node, const std::shared_ptr<MCode> & code);
  ~MotionDecision();
  void Config();
  bool Init(rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr response_msg_pub);

public:
  void DecideServoCmd(const MotionServoCmdMsg::SharedPtr & msg);
  template<typename CmdRequestT, typename CmdResponseT>
  void DecideResultCmd(const CmdRequestT request, CmdResponseT response);
  void DecideCustomCmd(
    const MotionCustomSrv::Request::SharedPtr request,
    MotionCustomSrv::Response::SharedPtr response);
  void DecideQueueCmd(
    const MotionQueueCustomSrv::Request::SharedPtr request,
    MotionQueueCustomSrv::Response::SharedPtr response);
  void DecideSequenceCmd(
    const MotionSequenceSrv::Request::SharedPtr request,
    MotionSequenceSrv::Response::SharedPtr response);
  inline void SetSequnceTotalDuration(int64_t sequence_total_duration)
  {
    handler_ptr_->SetSequnceTotalDuration(sequence_total_duration);
  }
  inline bool SelfCheck()
  {
    return handler_ptr_->SelfCheck();
  }

  inline void SetState(const MotionMgrState & state)
  {
    handler_ptr_->SetState(state);
  }

private:
  inline bool IsStateValid(int32_t motion_id, int32_t & error_code)
  {
    if (estop_) {
      if (motion_id != MotionIDMsg::ESTOP && motion_id != MotionIDMsg::RECOVERYSTAND) {
        ERROR("Forbidden ResultCmd(%d) when estop", motion_id);
        error_code = code_ptr_->GetCode(MotionCode::kEstop);
        return false;
      }
    }
    if (motion_id == MotionIDMsg::RECOVERYSTAND && laser_helper_->IsStuck()) {
      ERROR("Forbidden ResultCmd(%d) when stuck", motion_id);
      error_code = code_ptr_->GetCode(MotionCode::kStuck);
      return false;
    }
    return true;
  }
  inline bool IsStateValid(int32_t motion_id)
  {
    if (estop_) {
      return motion_id == MotionIDMsg::ESTOP ||
             motion_id == MotionIDMsg::RECOVERYSTAND;
    }
    return true;
  }
  inline bool IsStateValid()
  {
    return estop_;
  }
  inline void SetMode(DecisionStatus mode)
  {
    motion_work_mode_ = mode;
  }
  inline bool IsModeValid()
  {
    return true;
  }
  inline bool IsModeValid(int32_t cmd_source)
  {
    return cmd_source <= (int32_t)motion_work_mode_;
  }
  inline void ResetMode()
  {
    motion_work_mode_ = DecisionStatus::kIdle;
  }

private:
  void ServoStart(const MotionServoCmdMsg::SharedPtr & msg);
  void ServoData(const MotionServoCmdMsg::SharedPtr & msg);
  void ServoEnd(const MotionServoCmdMsg::SharedPtr & msg);
  void Update(MotionStatusMsg::SharedPtr & motion_status_ptr);
  bool WaitHandlingResult(int32_t motion_id, int32_t duration, int32_t & code);
  void StopMotion();
  void ServoResponseThread();
  inline void SetWorkStatus(const DecisionStatus & status_code)
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    motion_work_mode_ = status_code;
  }

  /**
   * @brief Get the Work Status object
   *
   * @return DecisionStatus
   */
  inline DecisionStatus GetWorkStatus()
  {
    std::unique_lock<std::mutex> lk(status_mutex_);
    return motion_work_mode_;
  }

  /**
   * @brief 消费一条伺服反馈计数
   *
   * @return true 消费正常
   * @return false 消费异常，此时消息队列析构或reset，需要注意后续代码安全
   */
  inline bool NeedServoResponse()
  {
    int unused_counter;
    return servo_response_queue_.DeQueue(unused_counter);
  }

  /**
   * @brief 生产一条伺服反馈计数
   *
   */
  inline void SetServoResponse()
  {
    servo_response_queue_.EnQueue(1);
  }

  /**
   * @brief 重置伺服反馈计数队列
   *        该行为会导致计数消费函数退出且返回false
   *
   */
  inline void ResetServoResponse()
  {
    servo_response_queue_.Reset();
  }

  /**
   * @brief 重置伺服反馈消息
   *
   */
  inline void ResetServoResponseMsg()
  {
    servo_response_msg_.result = true;
    servo_response_msg_.code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
  }

private:
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  std::shared_ptr<MotionHandler> handler_ptr_ {nullptr};
  DecisionStatus motion_work_mode_ {DecisionStatus::kIdle};
  std::mutex status_mutex_;
  std::shared_ptr<MCode> code_ptr_;

  /* Servo cmd members */
  std::thread servo_response_thread_;
  common::MsgQueue<int> servo_response_queue_;
  rclcpp::Publisher<MotionServoResponseMsg>::SharedPtr servo_response_pub_;
  MotionServoResponseMsg servo_response_msg_;
  std::shared_ptr<LaserHelper> laser_helper_;
  bool estop_ {false};
};  // class MotionDecision
}  // namespace motion
}  // namespace cyberdog

#endif  // MOTION_MANAGER__MOTION_DECISION_HPP_
