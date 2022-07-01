#!/usr/bin/env python3
# Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

import time, sys
import rclpy
from protocol.msg import MotionServoCmd
from rclpy.qos import QoSProfile

theta = (3.14 * 2 / 6) if len(sys.argv) == 1 else float(sys.argv[1]) 
omega = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1 
if omega < 0:
    print("Omega should to be positive")
omega = omega if theta > 0 else (-1 * omega)
duration = abs(theta / omega)
print(duration)
kMotionID = 303
kDataFrame = 1
kEndFrame = 2
rclpy.init()
qos = QoSProfile(depth=10)
node = rclpy.create_node('motion_angle')
pub = node.create_publisher(MotionServoCmd, 'motion_servo_cmd', qos)
servo_cmd = MotionServoCmd()
servo_cmd.motion_id = kMotionID
servo_cmd.step_height.fromlist([0.05, 0.05])
servo_cmd.vel_des.fromlist([0.0, 0.0, omega])
begin = time.time()
while(time.time() < begin + duration):
    pub.publish(servo_cmd)
    time.sleep(0.05)
servo_cmd.cmd_type = kEndFrame
pub.publish(servo_cmd)
