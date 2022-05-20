#!/usr/bin/env python3

from protocol.msg import MotionServoCmd
import rclpy
import datetime
from rclpy.node import Node
from std_msgs.msg import Float32

class ServoCmdSubscriber(Node):

    def __init__(self):
        super().__init__('servo_subscriber')
        self.last_timestamp = datetime.datetime.now().timestamp()
        self.subscription = self.create_subscription(
            MotionServoCmd,
            'motion_servo_cmd',
            self.topic_callback,
            10)
        self.publisher = self.create_publisher(Float32, "interval", 1)

    def topic_callback(self, msg):
        this_timestamp = datetime.datetime.now().timestamp()
        interval = this_timestamp - self.last_timestamp
        print(interval)
        interval_msg = Float32()
        interval_msg.data = interval
        self.publisher.publish(interval_msg)
        self.last_timestamp = this_timestamp

def main(args=None):
    rclpy.init(args=args)
    servo_subscriber = ServoCmdSubscriber()
    rclpy.spin(servo_subscriber)
    servo_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
