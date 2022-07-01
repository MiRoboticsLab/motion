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

import sys
import rclpy
from protocol.srv import MotionResultCmd

theta = (3.14 * 2 / 6) if len(sys.argv) == 1 else float(sys.argv[1]) 
kMotionID = 202
rclpy.init()
node = rclpy.create_node('motion_angle')
client = node.create_client(MotionResultCmd, 'motion_result_cmd')
req = MotionResultCmd.Request()
req.motion_id = kMotionID
req.pos_des.fromlist([theta, 0.0, 0.0])
req.duration = 1000
if not client.service_is_ready():
    print("MotionManager service is not ready")
    sys.exit()
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec= 2.0)
if future.result() is not None:
    print("Motion roll code: {}".format(future.result().code))
else:
    print("MotionManager service error")