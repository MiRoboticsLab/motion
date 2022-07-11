#include "protocol/lcm/heightmap_t.hpp"
#include "lcm/lcm-cpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_bridge/elevation_bridge.hpp"
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
    pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(
      "elevation_submap",
      rclcpp::SystemDefaultsQoS());
    lcm_->subscribe("local_heightmap", &AsController::LCMCallback, this);
    std::thread{[this]() {
        // while (0 == lcm_->handle()) {}}
        while (rclcpp::ok()) {
          while (0 == this->lcm_->handleTimeout(1000)) {
            ERROR("Cannot read LCM from bridge");
          }
        }
      }}.detach();
    length_x_ = SIZE_X * RESOLUTION;
    length_y_ = SIZE_Y * RESOLUTION;
  }
  ~AsController() {}
  void Spin()
  {
    rclcpp::spin(node_);
    rclcpp::shutdown();
  }

private:
  void LCMCallback(const lcm::ReceiveBuffer *, const std::string &, const heightmap_t * msg)
  {
    grid_map::GridMap map;
    map.setFrameId("map");
    map.add(LAYER_ELEVATION);
    grid_map::Position center(msg->robot_loc[0], msg->robot_loc[1]);
    map.setGeometry(grid_map::Length(length_x_, length_y_), RESOLUTION, center);
    grid_map::GridMapIterator iter(map); // NOTE 迭代器从左上角开始向下遍历，不同于SubmapIterator向右开始遍历。？
    for (int8_t j = 0; j < SIZE_Y; ++j) {
      for (int8_t i = 0; i < SIZE_X; ++i) {
        map.at(LAYER_ELEVATION, *iter) = msg->map[i][j];
        ++iter;
      }
    }
    std::unique_ptr<grid_map_msgs::msg::GridMap> map_ros;
    map_ros = grid_map::GridMapRosConverter::toMessage(map);
    pub_->publish(*map_ros);
  }
  std::shared_ptr<lcm::LCM> lcm_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  double length_x_, length_y_;
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
