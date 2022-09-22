#include "protocol/lcm/file_send_lcmt.hpp"
#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_bridge/file_bridge.hpp"
namespace cyberdog
{
namespace motion
{
class AsController
{
public:
  AsController()
  {
    node_.reset(new rclcpp::Node("as_controller"));
    lcm_.reset(new lcm::LCM(kLCMBirdgeSubscribeURL));
    lcm_->subscribe(kLCMBridgeFileChannel, &AsController::LCMCallback, this);
    std::thread{[this]() {
        while (rclcpp::ok()) {
          while (0 == this->lcm_->handleTimeout(1000)) {
            ERROR("Cannot read LCM from bridge");
          }
        }
      }
    }.detach();

  }
  ~AsController() {file_.close();}
  void Spin()
  {
    rclcpp::spin(node_);
    rclcpp::shutdown();
  }

private:
  void LCMCallback(const lcm::ReceiveBuffer *, const std::string &, const file_send_lcmt * msg)
  {
    INFO("%s", msg->data.c_str());
    // std::ofstream file("/home/harvey/test.toml");
    if(file_.is_open()) {
      file_ << msg->data;
    }
    // file_ << "\n";
  }
  std::shared_ptr<lcm::LCM> lcm_;
  rclcpp::Node::SharedPtr node_;
  std::ofstream file_{"/home/harvey/test.toml"};

};
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("AsController");
  std::shared_ptr<cyberdog::motion::AsController> ac =
    std::make_shared<cyberdog::motion::AsController>();
  ac->Spin();
}
