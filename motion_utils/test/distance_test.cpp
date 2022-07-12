#include "motion_utils/motion_utils.hpp"
#include "motion_action/motion_macros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  cyberdog::motion::MotionUtils& mu = cyberdog::motion::MotionUtils::GetMotionUtils();
  // mu.ExecuteWalkDuration(10000, 0.1, 0.2, 0.3);  
  mu.ExecuteWalkDistance(1.0, 0.1, 0.0, 0.0);  
}