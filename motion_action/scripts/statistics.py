#!/usr/bin/env python3

from cProfile import label
from glob import glob
import threading
import matplotlib.pyplot as plt
# plt.switch_backend('agg') 
from protocol.msg import MotionServoCmd
import rclpy
import datetime
from rclpy.node import Node
from threading import Thread

class ServoCmdSubscriber(Node):

    def __init__(self):
        super().__init__('servo_subscriber')
        self.timestamps = []
        self.last_timestamp = datetime.datetime.now().timestamp()
        fig, self.ax = plt.subplots()
        self.subscription = self.create_subscription(
            MotionServoCmd,
            'motion_servo_cmd',
            self.topic_callback,
            10)
        self.timer = self.create_timer(1, self.ploting)
        # self.thread = Thread(target=self.ploting_thread)
        # self.thread.start()

    def topic_callback(self, msg):
        this_timestamp = datetime.datetime.now().timestamp()
        self.timestamps.append(this_timestamp - self.last_timestamp)
        print(this_timestamp - self.last_timestamp)
        self.last_timestamp = this_timestamp
    
    def ploting(self):
        length = len(self.timestamps)
        if not length:
            return
        self.ax.cla()
        start = 0 if (length < 100) else (length - 100) ; 
        self.ax.bar(range(len(self.timestamps[start:])), label='time', height=self.timestamps[start:], width=0.3)
        self.ax.legend()
        plt.ylim([0, 0.5])
        plt.ylabel('s')
        plt.pause(0.01)

    def ploting_thread(self):
        timestamps = self.timestamps
        length = len(timestamps)
        if not length:
            plt.pause(0.01)
            return
        self.ax.cla()
        start = 0 if (length < 100) else (length - 100) ; 
        self.ax.bar(range(len(timestamps[start:])), label='time', height=timestamps[start:], width=0.3)
        self.ax.legend()
        plt.ylim([0, 0.5])
        plt.ylabel('s')
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    servo_subscriber = ServoCmdSubscriber()
    rclpy.spin(servo_subscriber)
    servo_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
