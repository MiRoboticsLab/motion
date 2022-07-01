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

import sys, time
import rclpy
from protocol.srv import MotionResultCmd
from protocol.msg import MotionServoCmd
from rclpy.qos import QoSProfile

def standup():
    kMotionID = 111
    rclpy.init()
    node = rclpy.create_node('motion')
    client = node.create_client(MotionResultCmd, 'motion_result_cmd')
    req = MotionResultCmd.Request()
    req.motion_id = kMotionID
    if not client.wait_for_service(5.0):
        node.get_logger().info("Waiting MotionManager service timeout for 5 seconds")
        sys.exit()
    if not client.service_is_ready():
        node.get_logger().info("MotionManager service is not ready")
        sys.exit()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec= 5.0)
    if future.result() is not None:
        node.get_logger().info("Motion standup code: {}".format(future.result().code))
    else:
        node.get_logger().info("MotionManager service error")
    rclpy.shutdown()

def getdown():
    kMotionID = 101
    rclpy.init()
    node = rclpy.create_node('motion')
    client = node.create_client(MotionResultCmd, 'motion_result_cmd')
    req = MotionResultCmd.Request()
    req.motion_id = kMotionID
    if not client.wait_for_service(5.0):
        node.get_logger().info("Waiting MotionManager service timeout for 5 seconds")
        sys.exit()
    if not client.service_is_ready():
        node.get_logger().info("MotionManager service is not ready")
        sys.exit()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec= 5.0)
    if future.result() is not None:
        node.get_logger().info("Motion getdown code: {}".format(future.result().code))
    else:
        node.get_logger().info("MotionManager service error")
    rclpy.shutdown()

def walk(x = 0.1, y = 0.0, duration = 10):
    kMotionID = 303
    kDataFrame = 1
    kEndFrame = 2
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('motion')
    pub = node.create_publisher(MotionServoCmd, 'motion_servo_cmd', qos)
    servo_cmd = MotionServoCmd()
    servo_cmd.motion_id = kMotionID
    servo_cmd.vel_des.fromlist([x, y, 0.0])
    servo_cmd.step_height.fromlist([0.05, 0.05])
    begin = time.time()
    while(time.time() < begin + duration):
        pub.publish(servo_cmd)
        node.get_logger().info("walking: {} {}".format(x, y))
        time.sleep(0.05)
    servo_cmd.cmd_type = kEndFrame
    pub.publish(servo_cmd)
    rclpy.shutdown()

def roll(theta = 3.14 * 2 / 6):
    kMotionID = 202
    rclpy.init()
    node = rclpy.create_node('motion')
    client = node.create_client(MotionResultCmd, 'motion_result_cmd')
    req = MotionResultCmd.Request()
    req.motion_id = kMotionID
    node.get_logger().info(theta)
    req.rpy_des.fromlist([theta, 0.0, 0.0])
    req.duration = 1000
    if not client.wait_for_service(5.0):
        node.get_logger().info("Waiting MotionManager service timeout for 5 seconds")
        sys.exit()
    if not client.service_is_ready():
        node.get_logger().info("MotionManager service is not ready")
        sys.exit()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec= 2.0)
    if future.result() is not None:
        node.get_logger().info("Motion roll code: {}".format(future.result().code))
    else:
        node.get_logger().info("MotionManager service error")
    rclpy.shutdown()

def pitch(theta = 3.14 * 2 / 6):
    kMotionID = 202
    rclpy.init()
    node = rclpy.create_node('motion')
    client = node.create_client(MotionResultCmd, 'motion_result_cmd')
    req = MotionResultCmd.Request()
    req.motion_id = kMotionID
    node.get_logger().info(theta)
    req.rpy_des.fromlist([0.0, theta, 0.0])
    req.duration = 1000
    if not client.wait_for_service(5.0):
        node.get_logger().info("Waiting MotionManager service timeout for 5 seconds")
        sys.exit()
    if not client.service_is_ready():
        node.get_logger().info("MotionManager service is not ready")
        sys.exit()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec= 2.0)
    if future.result() is not None:
        node.get_logger().info("Motion pitch code: {}".format(future.result().code))
    else:
        node.get_logger().info("MotionManager service error")
    rclpy.shutdown()

def yaw(theta = 3.14 * 2 / 6, omega = 0.1):
    if omega < 0:
        node.get_logger().info("Omega should to be positive")
    omega = omega if theta > 0 else (-1 * omega)
    duration = abs(theta / omega)
    node.get_logger().info(duration)
    kMotionID = 303
    kDataFrame = 1
    kEndFrame = 2
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('motion')
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
    rclpy.shutdown()
