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
    lcm_sub_.reset(new lcm::LCM(cyberdog::motion::PUBLISH_URL));
    lcm_pub_.reset(new lcm::LCM(cyberdog::motion::SUBSCRIBE_URL));
    lcm_sub_->subscribe("robot_control_cmd", &SimMotionController::HandleCmd, this);
    std::thread([this]() {while (0 == this->lcm_sub_->handle()) {}}).detach();
  }
  void Run()
  {
    while (true) {
      // lcm_sub_->handle();
      lcm_pub_->publish("robot_control_response", &res_);
      INFO("MotionController send res mode: %d, gait_id: %d", res_.mode, res_.gait_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

private:
  std::shared_ptr<lcm::LCM> lcm_sub_, lcm_pub_;
  robot_control_response_lcmt res_;
  void HandleCmd(const lcm::ReceiveBuffer *, const std::string &,
                 const robot_control_cmd_lcmt * msg)
  {
    INFO("MotionController get cmd mode %d, gait_id %d, life_count %d", msg->mode, msg->gait_id, msg->life_count);
    res_.mode = msg->mode;
    res_.gait_id = msg->gait_id;
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
