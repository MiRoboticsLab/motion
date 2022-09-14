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

#include <motion_action/motion_action.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <memory>
#include <string>

class SimMotionController
{
public:
  SimMotionController()
  {
    lcm_sub_.reset(new lcm::LCM(cyberdog::motion::kLCMActionPublishURL));
    lcm_pub_.reset(new lcm::LCM(cyberdog::motion::kLCMActionSubscibeURL));
    lcm_recv_pub_.reset(new lcm::LCM(cyberdog::motion::kLCMActionSubscibeURL));
    lcm_sub_->subscribe("robot_control_cmd", &SimMotionController::HandleCmd, this);
    lcm_sub_->subscribe("user_gait_file", &SimMotionController::HandleFileCmd, this);
    std::thread([this]() {while (0 == this->lcm_sub_->handle()) {}}).detach();
  }
  void Run()
  {
    while (true) {
      // lcm_sub_->handle();
      lcm_pub_->publish("robot_control_response", &res_);
      // INFO(
      //     "MotionController send res:\n mode: %d\n gait_id: %d\n contact: %d\n order_process_bar: %d\n switch_status: %d\n ori_error: %d\n footpos_error: %d\n motor_error: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]\n", res_.mode, res_.gait_id, res_.contact, res_.order_process_bar, res_.switch_status, res_.ori_error, res_.footpos_error,
      //     res_.motor_error[0], res_.motor_error[1], res_.motor_error[2], res_.motor_error[3],
      //     res_.motor_error[4], res_.motor_error[5], res_.motor_error[6], res_.motor_error[7],
      //     res_.motor_error[8], res_.motor_error[9], res_.motor_error[10], res_.motor_error[11]);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

private:
  std::shared_ptr<lcm::LCM> lcm_sub_, lcm_pub_, lcm_recv_pub_;
  robot_control_response_lcmt res_;
  file_recv_lcmt recv_result_;
  void HandleCmd(const lcm::ReceiveBuffer *, const std::string &,
                 const robot_control_cmd_lcmt * msg)
  {
    INFO("MotionController get cmd mode %d, gait_id %d, life_count %d", msg->mode, msg->gait_id, msg->life_count);
    res_.mode = msg->mode;
    res_.gait_id = msg->gait_id;
    res_.contact = msg->contact;
    res_.order_process_bar = 100;
    res_.switch_status = 0;
    res_.ori_error = 0;
    res_.footpos_error = 0;
    memset(res_.motor_error, 0, sizeof(res_.motor_error));
  }
  void HandleFileCmd(const lcm::ReceiveBuffer *, const std::string &,
                 const file_send_lcmt * msg)
  {
    INFO("MotionController get SequenceCmd data %s", msg->data.c_str());
    recv_result_.result = 0;
    lcm_recv_pub_->publish("user_gait_result", &recv_result_);
    INFO("publish result: %d", recv_result_.result);
  }
  LOGGER_MINOR_INSTANCE("SimMotionController")
};

void SimMotionControllerFunction()
{
  SimMotionController smc;
  smc.Run();
}

int main()
{
  LOGGER_MAIN_INSTANCE("test_as_controller")
  std::thread t2(SimMotionControllerFunction);

  if (t2.joinable()) {
    t2.join();
  }
}
