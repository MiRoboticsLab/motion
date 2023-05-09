# Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from matplotlib.pyplot import text
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, LocalSubstitution, PythonExpression
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
  preset_cmd = LaunchConfiguration('cmd')
  sim_cmd = LaunchConfiguration('sim')
  declare_cmd_arg = DeclareLaunchArgument('cmd', default_value='1.toml')
  declare_sim_arg = DeclareLaunchArgument('sim', default_value='False')
  cmd_preset = PathJoinSubstitution([get_package_share_directory('motion_action'), 'preset', preset_cmd])
  
  start_test_as_manager_cmd = Node(
    package='motion_action',
    executable='test_as_manager',
    name='test_as_manager',
    output='screen',
    parameters=[
      {'publish_url' : ''},
      {'subscribe_url' : ''},
      {'cmd_preset' : cmd_preset}]
  )
  ld = LaunchDescription()
  ld.add_action(declare_cmd_arg)
  ld.add_action(declare_sim_arg)
  ld.add_action(start_test_as_manager_cmd)
  return ld