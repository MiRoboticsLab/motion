/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#ifndef GRADIENT_LAYER_HPP_
#define GRADIENT_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_costmap_2d/grid_map_costmap_2d.hpp"

namespace cyberdog_gridmap_costmap_plugin
{

class GridMapCostLayer : public nav2_costmap_2d::Layer
{
public:
  GridMapCostLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

private:
  /// @brief Used for the observation message filters
  std::vector<std::shared_ptr<message_filters::SubscriberBase>> observation_subscribers_;
  /// @brief Used to make sure that transforms are available for each sensor
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  std::string global_frame_;

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr mapSubscriber_;
void
  ScanCallback(
  grid_map_msgs::msg::GridMap::ConstSharedPtr message);
  grid_map::GridMap map_;
  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;
  double min_pass_threshold_;
  double max_pass_threshold_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
};

}  // namespace cyberdog_gridmap_costmap_plugin

#endif  // GRADIENT_LAYER_HPP_
