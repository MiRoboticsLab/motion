
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

#ifndef CYBERDOG_AFT__MOTION__LIDAR_UTILS_HPP_
#define CYBERDOG_AFT__MOTION__LIDAR_UTILS_HPP_

#include <type_traits>
#include <chrono>
#include <vector>

#include "sensor_msgs/msg/laser_scan.hpp"

namespace cyberdog
{
namespace motion
{

namespace lidar_utils
{
inline double GetDistance(sensor_msgs::msg::LaserScan::SharedPtr msgs, double begin, double end)
{
  double average = 0.0;
  for (auto it = msgs->ranges.begin() + begin; it != msgs->ranges.begin() + end; it++) {
    average += *it;
  }

  return average / (end - begin);
}

inline double GetLeftAngleDiff(sensor_msgs::msg::LaserScan::SharedPtr msgs, 
  double front_begin, double front_end, double rear_begin, double rear_end)
{
  double front_average = 0.0, rear_average = 0.0;
  for (auto it = msgs->ranges.begin() + front_begin; it != msgs->ranges.begin() + front_end; it++) {
    front_average += *it;
  }
  front_average /= (front_end - front_begin);
  for (auto it = msgs->ranges.begin() + rear_begin; it != msgs->ranges.begin() + rear_end; it++) {
    rear_average += *it;
  }
  rear_average /= (rear_end - rear_begin);
  INFO("%f, %f", front_average, rear_average);
  return front_average - rear_average*2;
}

}  // lidar_utils

}  // namespace motion
}  // namespace cyberdog

#endif  // CYBERDOG_AFT__MOTION__LIDAR_UTILS_HPP_
