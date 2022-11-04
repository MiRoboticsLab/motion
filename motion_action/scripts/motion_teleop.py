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
Moving around:
   u    i    o  [ ]
   j    k    l
   m    ,    .

[/] : linear y speed
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':( 1.0,  0.0,  0.0),
        'o':( 1.0, -1.0,  0.0),
        'j':( 0.0,  1.0,  0.0),
        'l':( 0.0, -1.0,  0.0),
        'u':( 1.0,  1.0,  0.0),
        ',':(-1.0,  0.0,  0.0),
        '.':(-1.0,  1.0,  0.0),
        'm':(-1.0, -1.0,  0.0),
        '[':( 0.0,  0.0,  1.0),
        ']':( 0.0,  0.0, -1.0),
           }

speedBindings={
        'q':( 1.1, 1.1),
        'z':( 0.9, 0.9),
        'w':( 1.1, 1.0),
        'x':( 0.9, 1.0),
        'e':( 1.0, 1.1),
        'c':( 1.0, 0.9),
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
    end = None
    opts = None
    argv = sys.argv[1:]
    try:
        opts, args = getopt.getopt(argv, "e:")  # 短选项模式
    except:
        print("Error")
    if opts != None:
        for opt, arg in opts:
            if opt in ['-e']:
                end = arg

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
    motion_id = 0
    life_count = 0
    speed = .2
    speed_y = .2
    turn = 1
    x = 0.0
    y = 0.0
    th = 0.0
    target_speed = 0.0
    target_speed_y = 0.0
    target_turn = 0.0
    control_speed = 0.0
    control_speed_y = 0.0
    control_turn = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while(1):
            rclpy.spin_once(node=node, timeout_sec=0)
            # print(auto)
            # print(triggered)
            if not triggered:
                sleep(0.1)
                continue
            motion_id = MotionID.WALK_USERTROT
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][2]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                speed_y = speed_y * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ' or key == 'k' :
                x = 0.0
                y = 0.0
                th = 0.0
                control_speed = 0.0
                control_speed_y = 0.0
                control_turn = 0.0
            else:
                count = count + 1
                if count > 4:
                    x = 0.0
                    y = 0.0
                    th = 0.0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_speed_y = speed_y * y
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_speed_y > control_speed_y:
                control_speed_y = min( target_speed_y, control_speed_y + 0.02 )
            elif target_speed_y < control_speed_y:
                control_speed_y = max( target_speed_y, control_speed_y - 0.02 )
            else:
                control_speed_y = target_speed_y

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            cmd = MotionServoCmd()
            cmd.motion_id = motion_id; 
            #   cmd.mode = mode; cmd.gait_id = gait_id; cmd.life_count = life_count + 1
            cmd.vel_des.fromlist([control_speed, control_speed_y, control_turn])
            cmd.pos_des.fromlist([0.0, 0.0, 0.2])
            cmd.step_height.fromlist([0.05, 0.05])
            cmd.value = 2
            #   if key in moveBindings.keys():
            pub.publish(cmd)

    except Exception as e:
        print(e)

    finally:
        if end:
            cmd = MotionServoCmd()
            cmd.cmd_type = MotionServoCmd.SERVO_END
            # cmd.motion_id = MotionID.FORCECONTROL_RELATIVEYLY
            # cmd.mode = 3; cmd.gait_id = 0; cmd.life_count = life_count + 1
            # cmd.pos_des.fromlist([0.0, 0.0, 0.3])
            # cmd.vel_des.fromlist([0.0, 0.0, 0.0])
            pub.publish(cmd)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
