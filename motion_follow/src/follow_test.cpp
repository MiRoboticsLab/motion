#include "motion_follow/command.hpp"


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  cyberdog::motion::Command<cyberdog::motion::FollowWall> c;
  while (rclcpp::ok()) {
    c.Run();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}