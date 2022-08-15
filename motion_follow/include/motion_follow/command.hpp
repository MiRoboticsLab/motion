// Copyright (c) 2021 Xiaomi Corporation
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

#ifndef CYBERDOG_AFT__MOTION__COMMMAND_HPP_
#define CYBERDOG_AFT__MOTION__COMMMAND_HPP_

#include <chrono>
#include <memory>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/msg/motion_servo_response.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
// #include "cyberdog_aft/motion/PID.hpp"
#include "motion_follow/lidar_utils.hpp"

namespace cyberdog
{
namespace motion
{

using namespace std::chrono_literals;

struct command_follow_wall_tag{};
struct command_follow_line_tag{};

struct FollowWall {
  using command_type = command_follow_wall_tag;
};

struct FollowLine {
  using command_type = command_follow_line_tag;
};

template <typename CommandType>
class Command
{
public:
  using Type = typename CommandType::command_type;

  enum class FollowState
  {
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    SHIFT_LEFT,
    SHIFT_RIGHT,
    TURNING,
  };

  Command() // : controller_ {3.0, 0, 5}
  {
    node_ptr_ = rclcpp::Node::make_shared("aft_motion_lidar_data");

    // parameters
    node_ptr_->declare_parameter<double>("parameter_kp", 5.0f);
    node_ptr_->declare_parameter<double>("parameter_ki", 0.0f);
    node_ptr_->declare_parameter<double>("parameter_kd", 0.0f);

    lidar_sub_ = node_ptr_->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SystemDefaultsQoS(),
      std::bind(&Command::HandleLidarSensorMsgs, this, std::placeholders::_1));

    // odom_sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
    //   "odom_out", rclcpp::SystemDefaultsQoS(),
    //   std::bind(&Command::HandleOdometryMsgs, this, std::placeholders::_1));

    motion_command_pub_ = node_ptr_->create_publisher<::protocol::msg::MotionServoCmd>(
      "motion_servo_cmd", rclcpp::SystemDefaultsQoS());

    scan_filterd_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::LaserScan>("scan_filterd", 1);

    // pid_error_pub_ = node_ptr_->create_publisher<std_msgs::msg::Float32>(
    //   "pid_error", rclcpp::SystemDefaultsQoS());

    // timer_ = node_ptr_->create_wall_timer(
    //   1000ms, std::bind(&Command::RefreshParameters, this));

    spin_thread_ptr_ = std::make_shared<std::thread>(&Command::Spin, this);
  }

  void Run()
  {
    Run(Type{});
  }

  void Run(const command_follow_wall_tag& tag)
  {
    // controller_.SetParameters(parameter_kp_, parameter_ki_, parameter_kd_);
    // double dt = std::chrono::duration_cast<std::chrono::seconds>(
    //   std::chrono::high_resolution_clock::now() - start_).count();

    // constexpr double target_distance = 0.35; // 0.35 meter
    // double error = current_distance_.load() - target_distance;
    // double feedback = controller_.Update(error, dt);
    // Publish(error);
    // SetCommandVelocity(feedback);

    if (CheckStop(current_center_distance_)) {
      INFO("Front is wall , it's must stop now.");
      auto start = std::chrono::high_resolution_clock::now();
      double theta = 0.0f;
      while (rclcpp::ok()) {
          constexpr double linear_velocity = 0.0f;
          constexpr double angular_velocity = 0.2f;

          // double dt = std::chrono::duration_cast<std::chrono::seconds>(
          //   std::chrono::high_resolution_clock::now() - start).count();

          // theta += dt * angular_velocity;
          // INFO("theta : %f", theta);
          // if (theta > 6.28) {
          //   SetCommandVelocity(0.2f, 0.0f);
          //   break;
          // }

          constexpr double right_distance_threshold = 0.3f; // 0.3 meter
          constexpr double front_distance_threshold = 0.4f; // 0.4 meter

          if (current_center_distance_ >= front_distance_threshold && 
              current_right_distance_ >= right_distance_threshold) {
            SetCommandVelocity(0.2f, 0.0f);
            // state_ = FollowState::FORWARD;
            break;
          }

          SetCommandVelocity(linear_velocity, angular_velocity);
          // state_ = FollowState::TURNING;
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    }

    ControlInCorridor(current_left_distance_, current_right_distance_);
  }

  void Run(const command_follow_line_tag& tag)
  {
  }

private:
  void HandleLidarSensorMsgs(sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // INFO("scan size : %d", msg->ranges.size());
    sensor_msgs::msg::LaserScan msg_filtered;
    msg_filtered.ranges.assign(msg->ranges.begin()+505, msg->ranges.begin()+1515);
    msg_filtered.header = msg->header;
    msg_filtered.angle_increment = msg->angle_increment;
    msg_filtered.angle_max = 1.57;
    msg_filtered.angle_min = -1.57;
    msg_filtered.range_max = msg->range_max;
    msg_filtered.range_min = msg->range_min;
    msg_filtered.scan_time = msg->scan_time;
    msg_filtered.intensities = std::vector<float>(1010, 0);
    scan_filterd_pub_->publish(msg_filtered);

    current_left_angle_diff_ = lidar_utils::GetLeftAngleDiff(msg, 835, 845, 505, 515);
    current_left_distance_ = lidar_utils::GetDistance(msg, 400, 500);
    current_right_distance_ = lidar_utils::GetDistance(msg, 1500, 1600);
    current_center_distance_ = lidar_utils::GetDistance(msg, 900, 1100);
  }

  void HandleOdometryMsgs(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_odometry_ = *msg;
  }

  void SetCommandVelocity(const double & linear_velocity, const double & angular_velocity)
  {
    // INFO("SetCommandVelocity");
    std::vector<float> vel_des {
          linear_velocity,
          0.0,
          angular_velocity
        };

     std::vector<float> step_height {
          0.05,
          0.05
        };

    ::protocol::msg::MotionServoCmd command;
    command.motion_id = 305;
    command.vel_des = vel_des;
    command.step_height = step_height;
    motion_command_pub_->publish(command);
  }

  void SetCommandVelocity(::protocol::msg::MotionServoCmd & command, const double & linear_velocity, 
    const double & shift_velocity, const double & angular_velocity)
  {
    // INFO("SetCommandVelocity");
    std::vector<float> vel_des {
          linear_velocity,
          shift_velocity,
          angular_velocity
        };

     std::vector<float> step_height {
          0.05,
          0.05
        };

    command.motion_id = 301;
    command.vel_des = vel_des;
    command.value = 2;
    command.step_height = step_height;
  }

  void ControlInCorridor(const double & left_dist, const double & right_dist)
  {
    INFO("left dist: %f", left_dist);
    INFO("right dist: %f", right_dist);
    INFO("center dist: %f", current_center_distance_);

    // constexpr double target_distance = 0.35; // 0.35 meter
    // double angular_velocity = robot_odometry_.twist.twist.angular.z;
    // if (left_dist > target_distance && right_dist > target_distance) {
    //   SetCommandVelocity(0.2f, 0.0f);
    //   return;
    // }

    double delta_dist = left_dist - right_dist;
    constexpr double lower_limit = -0.1f; // 0.3 meter
    constexpr double upper_limit = 0.1f;  // 0.3 meter
    constexpr float vel_linear = 0.6;
    constexpr float vel_shift = 0.1;
    constexpr float vel_omega = 0.1;
    double angular_velocity = robot_odometry_.twist.twist.angular.z;

    // if (delta_dist >= lower_limit && lower_limit <= upper_limit) {
    //   SetCommandVelocity(0.2f, angular_velocity);
    //   return;
    // }

    // if (delta_dist < lower_limit) {
    //   angular_velocity -= 0.08;
    // }  else if (delta_dist > upper_limit) {
    //   angular_velocity += 0.08;
    // }

    // SetCommandVelocity(0.25f, angular_velocity);
    ::protocol::msg::MotionServoCmd command;
    INFO("------------");
    switch (state_) {
      case FollowState::FORWARD:
        SetCommandVelocity(command, vel_linear, 0.0, 0.0);
        INFO("Forward: %f", current_left_angle_diff_);
        if(current_left_angle_diff_ > 0.05) {
          state_ = FollowState::TURN_LEFT;
        } else if (current_left_angle_diff_ < -0.05) {
          state_ = FollowState::TURN_RIGHT;
        } else if (delta_dist > upper_limit) {
          state_ = FollowState::SHIFT_LEFT;
        } else if (delta_dist < lower_limit) {
          state_ = FollowState::SHIFT_RIGHT;
        }
        break;

      case FollowState::TURN_LEFT:
        SetCommandVelocity(command, vel_linear, 0.0, vel_omega);
        INFO("Turning left: %f", current_left_angle_diff_);
        if(current_left_angle_diff_ >= -0.05 && current_left_angle_diff_ <= 0.05) {
          state_ = FollowState::FORWARD;
        } else if (current_left_angle_diff_ < -0.05) {
          state_ = FollowState::TURN_RIGHT;
        }
        break;

      case FollowState::TURN_RIGHT:
        SetCommandVelocity(command, vel_linear, 0.0, -vel_omega);
        INFO("Turning right: %f", current_left_angle_diff_);
        if(current_left_angle_diff_ >= -0.05 && current_left_angle_diff_ <= 0.05) {
          state_ = FollowState::FORWARD;
        } else if (current_left_angle_diff_ > 0.05) {
          state_ = FollowState::TURN_LEFT;
        }

        break;

      case FollowState::SHIFT_LEFT:
        SetCommandVelocity(command, vel_linear, vel_shift, 0.0);
        if(abs(delta_dist) < upper_limit) {
          state_ = FollowState::FORWARD;
        }
        break;

      case FollowState::SHIFT_RIGHT:
        SetCommandVelocity(command, vel_linear, -vel_shift, 0.0);
        if(abs(delta_dist) < upper_limit) {
          state_ = FollowState::FORWARD;
        }
        break;

      case FollowState::TURNING:
        break;

      default:
        break;
    }
    motion_command_pub_->publish(command);

    // INFO("SetCommandVelocity");
    // std::vector<float> vel_des {
    //       0.2,
    //       robot_odometry_.twist.twist.linear.y,
    //       angular_velocity
    //     };

    // std::vector<float> step_height {
    //     0.05,
    //     0.05
    //   };

    // ::protocol::msg::MotionServoCmd command;
    // command.motion_id = 303;
    // command.vel_des = vel_des;
    // command.step_height = step_height;
    // motion_command_pub_->publish(command);
  }

  bool CheckStop(const double & center_dist) 
  {
    constexpr double distance_threshold = 0.32f; // 0.4 meter
    if (center_dist <= distance_threshold) {
      // INFO("SetCommandVelocity");
      return true;
    }

    return false;
  }

  void RefreshParameters()
  {
    node_ptr_->get_parameter("parameter_kp", parameter_kp_);
    node_ptr_->get_parameter("parameter_ki", parameter_ki_);
    node_ptr_->get_parameter("parameter_kd", parameter_kd_);

    INFO("parameter_kp = %f, parameter_ki = %f, parameter_kd = %f",
      parameter_kp_, parameter_ki_, parameter_kd_);
  }

  void Spin()
  {
    rclcpp::spin(node_ptr_);
  }

  // void Publish(double data)
  // {
  //   std_msgs::msg::Float32 pid_error;
  //   pid_error.data = data;
  //   pid_error_pub_->publish(pid_error);
  // }

  std::shared_ptr<rclcpp::Node> node_ptr_ {nullptr};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_ {nullptr};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_ {nullptr};
  rclcpp::Publisher<::protocol::msg::MotionServoCmd>::SharedPtr motion_command_pub_ {nullptr};
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pid_error_pub_ {nullptr};
  std::shared_ptr<std::thread> spin_thread_ptr_ {nullptr};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_filterd_pub_ {nullptr};
  // PID<double> controller_;
  // std::atomic<double> current_center_distance_;
  // std::atomic<double> current_left_distance_;
  // std::atomic<double> current_right_distance_;
  // std::atomic<double> current_left_angle_diff_;

  double current_center_distance_;
  double current_left_distance_;
  double current_right_distance_;
  double current_left_angle_diff_;

  nav_msgs::msg::Odometry robot_odometry_; 
  std::chrono::time_point<std::chrono::system_clock> start_ = std::chrono::high_resolution_clock::now();

  FollowState state_;

  // parameters
  double parameter_kp_ {0.0f};
  double parameter_ki_ {0.0f};
  double parameter_kd_ {0.0f};
  // rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motion
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__MOTION__COMMMAND_HPP_
