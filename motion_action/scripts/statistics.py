#!/usr/bin/env python3

from cProfile import label
from glob import glob
from sqlite3 import Timestamp
import threading
import matplotlib.pyplot as plt
# plt.switch_backend('agg') 
from protocol.msg import MotionServoCmd
import rclpy
import datetime
from rclpy.node import Node
from threading import Thread

timestamps =[]
class ServoCmdSubscriber(Node):

    def __init__(self):
        super().__init__('servo_subscriber')
        self.last_timestamp = datetime.datetime.now().timestamp()
        fig, self.ax = plt.subplots()
        self.subscription = self.create_subscription(
            MotionServoCmd,
            'motion_servo_cmd',
            self.topic_callback,
            10)
        # self.timer = self.create_timer(1, self.ploting)

    def topic_callback(self, msg):
        global timestamps
        this_timestamp = datetime.datetime.now().timestamp()
        # self.timestamps.append(this_timestamp - self.last_timestamp)
        timestamps.append(this_timestamp - self.last_timestamp)
        print(this_timestamp - self.last_timestamp)
        self.last_timestamp = this_timestamp
    
    def ploting(self):
        global timestamps
        length = len(timestamps)
        if not length:
            return
        self.ax.cla()
        start = 0 if (length < 100) else (length - 100) ; 
        self.ax.bar(range(len(timestamps[start:])), label='time', height=timestamps[start:], width=0.3)
        self.ax.legend()
        plt.ylim([0, 0.5])
        plt.ylabel('s')
        plt.pause(0.01)

def sub_thread(args=None):
    rclpy.init(args=args)
    servo_subscriber = ServoCmdSubscriber()
    rclpy.spin(servo_subscriber)
    servo_subscriber.destroy_node()
    rclpy.shutdown()

def main(args=None):
    thread = Thread(target=sub_thread)
    thread.start()
    fig, ax = plt.subplots()
    global timestamps
    while(rclpy.ok):
        length = len(timestamps)
        if not length:
            plt.pause(0.01)
            continue
        ax.cla()
        start = 0 if (length < 100) else (length - 100) ; 
        ax.bar(range(len(timestamps[start:])), label='time', height=timestamps[start:], width=0.3)
        ax.legend()
        plt.ylim([0, 0.5])
        plt.ylabel('s')
        plt.pause(0.01)

if __name__ == '__main__':
    main()
