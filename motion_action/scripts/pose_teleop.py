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

from concurrent.futures import thread
from doctest import register_optionflag
import os
import select
import sys, tty, termios, getopt
from time import sleep
import threading
# from typing import Protocol
import rclpy

from geometry_msgs.msg import Twist
# from protocol.msg import MotionCmd
from protocol.msg import MotionServoCmd
from protocol.msg import MotionID
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool

msg = """
Control Your CyberDog!
---------------------------
Height and Yaw:
        w     
   a    s    d
            

Pitch and Roll:
   u    i    o
   j    k    l
   m    ,    .

CTRL-C to quit
"""

moveBindings = {
        'w':( 1.0,  0.0,  0.0,  0.0),
        's':(-1.0,  0.0,  0.0,  0.0),
        'a':( 0.0,  0.0,  0.0,  1.0),
        'd':( 0.0,  0.0,  0.0, -1.0),
        'i':( 0.0,  0.0,  1.0,  0.0),
        ',':( 0.0,  0.0, -1.0,  0.0),
        'j':( 0.0,  1.0,  0.0,  0.0),
        'l':( 0.0, -1.0,  0.0,  0.0),
        'u':( 0.0,  1.0,  1.0,  0.0),
        'm':( 0.0,  1.0, -1.0,  0.0),
        'o':( 0.0, -1.0,  1.0,  0.0),
        '.':( 0.0, -1.0, -1.0,  0.0),
           }

triggered = True

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def switch_run():
    return

def keep_idle():
    return

def service_callback(request:SetBool.Request, response:SetBool.Response):
    global triggered
    triggered = request.data
    # print(triggered)
    response.success = True
    return response

def main(args=None):
    rclpy.init(args=args)
    # end = False
    motion_id = MotionID.FORCECONTROL_DEFINITIVELY
    # opts = None
    # argv = sys.argv[1:]
    # try:
    #     opts, args = getopt.getopt(argv, "ei:")  # 短选项模式
    # except:
    #     print("Error")
    # if opts != None:
    #     for opt, arg in opts:
    #         if opt in ['-e']:
    #             end = True
    #         if opt in ['-i']:
    #             motion_id = int(arg)
    global settings
    global triggered
    # print(auto)
    # print(triggered)
    settings = termios.tcgetattr(sys.stdin)
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(MotionServoCmd, 'motion_servo_cmd', qos)
    service = node.create_service(SetBool, 'tracking_command', service_callback)
    status = 0
    count = 0
    life_count = 0
    max_pos_z = 0.32
    min_pos_z = 0.15
    default_pos_z = 0.235
    max_roll = 0.52
    max_pitch = 0.3
    min_pitch = -0.25
    max_yaw = 0.65
    pos_z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    target_pos_z = 0.0
    target_pitch = 0.0
    target_yaw = 0.0
    control_pos_z = default_pos_z
    control_roll = 0.0
    control_pitch = 0.0
    control_yaw = 0.0
    resolution = 0.02

    try:
        print(msg)
        print("currently: pos_z [%s, %s] r [%s, %s] p [%s, %s] y [%s, %s]" %
            (max_pos_z, min_pos_z, max_roll, -max_roll, max_pitch, min_pitch, max_roll, -max_roll))
        while(1):
            rclpy.spin_once(node=node, timeout_sec=0)
            # print(auto)
            # print(triggered)
            if not triggered:
                sleep(0.1)
                continue
            key = getKey()
            if key in moveBindings.keys():
                pos_z = moveBindings[key][0]
                roll = moveBindings[key][1]
                pitch = moveBindings[key][2]
                yaw = moveBindings[key][3]
                count = 0
            elif key == ' ' or key == 'k' :
                pos_z = 0.0
                roll = 0.0
                pitch = 0.0
                yaw = 0.0
                control_pos_z = default_pos_z
                control_yaw = 0.0
                control_pitch = 0.0
                control_roll = 0.0
            else:
                count = count + 1
                # if count > 4:
                #     pos_z = 0.0
                #     roll = 0.0
                #     pitch = 0.0
                #     yaw = 0.0
                if (key == '\x03'):
                    break

            if count == 0:
                if pos_z > 0:
                    target_pos_z = max_pos_z * pos_z
                elif pos_z < 0:
                    target_pos_z = -min_pos_z * pos_z
                elif pos_z == 0:
                    target_pos_z = default_pos_z
                target_roll = max_roll * roll
                if pitch > 0:
                    target_pitch = max_pitch * pitch
                elif pitch < 0:
                    target_pitch = -min_pitch * pitch
                elif pitch == 0:
                    target_pitch = 0.0
                target_yaw = max_yaw * yaw

                if target_pos_z > control_pos_z:
                    control_pos_z = min( target_pos_z, control_pos_z + resolution )
                elif target_pos_z < control_pos_z:
                    control_pos_z = max( target_pos_z, control_pos_z - resolution )
                else:
                    control_pos_z = target_pos_z

                if target_roll > control_roll:
                    control_roll = min( target_roll, control_roll + resolution )
                elif target_roll < control_roll:
                    control_roll = max( target_roll, control_roll - resolution )
                else:
                    control_roll = target_roll

                if target_pitch > control_pitch:
                    control_pitch = min( target_pitch, control_pitch + resolution )
                elif target_pitch < control_pitch:
                    control_pitch = max( target_pitch, control_pitch - resolution )
                else:
                    control_pitch = target_pitch

                if target_yaw > control_yaw:
                    control_yaw = min( target_yaw, control_yaw + resolution )
                elif target_yaw < control_yaw:
                    control_yaw = max( target_yaw, control_yaw - resolution )
                else:
                    control_yaw = target_yaw

            cmd = MotionServoCmd()
            cmd.motion_id = motion_id
            cmd.cmd_source = -1 # 最高调试优先级
            #   cmd.mode = mode; cmd.gait_id = gait_id; cmd.life_count = life_count + 1
            cmd.vel_des.fromlist([0.0, 0.0, 0.0])
            cmd.pos_des.fromlist([0.0, 0.0, control_pos_z])
            cmd.rpy_des.fromlist([control_roll, control_pitch, control_yaw])
            # cmd.step_height.fromlist([0.05, 0.05])
            # cmd.value = 2
            #   if key in moveBindings.keys():
            pub.publish(cmd)

    except Exception as e:
        print(e)

    # finally:
    #     if end:
    #         cmd = MotionServoCmd()
    #         cmd.cmd_type = MotionServoCmd.SERVO_END
    #         cmd.motion_id = motion_id
    #         cmd.cmd_source = -1 # 最高调试优先级
    #         # cmd.mode = 3; cmd.gait_id = 0; cmd.life_count = life_count + 1
    #         # cmd.pos_des.fromlist([0.0, 0.0, 0.3])
    #         # cmd.vel_des.fromlist([0.0, 0.0, 0.0])
    #         pub.publish(cmd)
    #         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
