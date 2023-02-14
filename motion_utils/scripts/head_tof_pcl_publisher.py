#!/usr/bin/env python3

import rclpy 
import time
import std_msgs.msg as std_msgs
from rclpy.node import Node
from std_msgs.msg import String
import sensor_msgs.msg as sensor_msgs
import numpy as np
import math
import threading

import lcm
import tof_lcmt
import robot_control_response_lcmt

from protocol.msg import HeadTofPayload    # CHANGE
from protocol.msg import RearTofPayload    # CHANGE
from protocol.msg import SingleTofPayload    # CHANGE

cos_array_scale = [-7,-5,-3,-1,1,3,5,7]
sin_array_scale = [7,5,3,1,-1,-3,-5,-7]
r2_array = [
  98,74,58,50,50,58,74,98,
  74,50,34,26,26,34,50,74,
  58,34,18,10,10,18,34,58,
  50,26,10,2 ,2 ,10,26,50,
  50,26,10,2, 2, 10,26,50,
  58,34,18,10,10,18,34,58,
  74,50,34,26,26,34,50,74,
  98,74,58,50,50,58,74,98
  ]
angle_arr = [
  25.8, 21.4, 19.2, 17.9, 17.9, 19.2, 21.4, 25.8,
  21.4, 18.5, 14.9, 13.1, 13.1, 14.9, 18.5, 21.4,
  19.2, 14.9, 11.1, 8.2,  8.2,  11.1, 14.9, 19.2,
  17.9, 13.1, 8.2,  3.7,  3.7,  8.2,  13.1, 17.9,
  17.9, 13.1, 8.2,  3.7,  3.7,  8.2,  13.1, 17.9,
  19.2, 14.9, 11.1, 8.2,  8.2,  11.1, 14.9, 19.2,
  21.4, 18.5, 14.9, 13.1, 13.1, 14.9, 18.5, 21.4,
  25.8, 21.4, 19.2, 17.9, 17.9, 19.2, 21.4, 25.8
  ]

def GetRotationMatrix(theta_x, theta_y, theta_z):
	sx = np.sin(theta_x)
	cx = np.cos(theta_x)
	sy = np.sin(theta_y)
	cy = np.cos(theta_y)
	sz = np.sin(theta_z)
	cz = np.cos(theta_z)
	return np.array([
        [cy*cz, cz*sx*sy-cx*sz, sx*sz+cx*cz*sy],
        [cy*sz, cx*cz+sx*sy*sz, cx*sy*sz-cz*sx],
        [-sy, cy*sx, cx*cy]])



h_left_t = np.array([0.259, 0.03, 0.102])
h_left_R = GetRotationMatrix(0.296, -0.266, 0.0)

h_right_t = np.array([0.259, -0.03, 0.102])
h_right_R = GetRotationMatrix(-0.296, -0.266, 0.0)

r_left_t = np.array([-0.021, 0.042, -0.051])
r_left_R = GetRotationMatrix(0.296, 0.0, 0.0)

r_right_t = np.array([-0.021, -0.042, -0.051])
r_right_R = GetRotationMatrix(-0.296, 0.0, 0.0)

lc=lcm.LCM("udpm://239.255.76.67:7667?ttl=255")
lcm_publish_flag = True

def msg_handler(channel, data):
    global lcm_publish_flag
    # message handling code here.  For example:
    msg = robot_control_response_lcmt.robot_control_response_lcmt().decode(data)
    if msg.mode == 12 or ( msg.mode == 11 and msg.gait_id >= 120 ) or msg.mode == 6:
        lcm_publish_flag = True
    else:
        lcm_publish_flag = False


def rec_responce():
    lc_r=lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
    lc_r.subscribe("robot_control_response", msg_handler)
    try:
        while True:
            lc_r.handle()
            time.sleep( 0.01 )
    except KeyboardInterrupt:
        pass

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('head_tof_pc_publisher')
        self.head_subscription = self.create_subscription(
            HeadTofPayload,
            'head_tof_payload',
            self.head_listener_callback,
            10)

        self.rear_subscription = self.create_subscription(
            RearTofPayload,
            'rear_tof_payload',
            self.rear_listener_callback,
            10)
        self.lock = threading.Lock()

        # self.pcd_topic = []
        # self.pcd_topic.append('left_head_pcd')
        # self.pcd_topic.append('right_head_pcd')
        # self.pcd_topic.append('left_rear_pcd')
        # self.pcd_topic.append('right_rear_pcd')
        
        # self.pcd_publisher = []
        # for i in range(0,4):
        #     pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, self.pcd_topic[i], 10)
        #     self.pcd_publisher.append(pcd_publisher)
        self.head_pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, "head_pc", 10)
        self.rear_pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, "rear_pc", 10)

        rec_thread = threading.Thread(target=rec_responce)
        rec_thread.start()


    # def tof_pcd_generate(self,tof_msg):
    #     tof_position = tof_msg.tof_position
    #     tof_time = tof_msg.header.stamp.nanosec/1000000 + tof_msg.header.stamp.sec * 1000
    #     for idx in range(0,64):
    #         z_array[idx] =  -tof_msg.data[63-idx]
    #         r_array[idx] = -z_array[idx]/math.cos(math.pi*angle_arr[idx]/180)*math.sin(math.pi*angle_arr[idx]/180)
    #         if tof_position == SingleTofPayload.RIGHT_HEAD or tof_position == SingleTofPayload.LEFT_HEAD:
    #             x_array[idx] = round(r_array[idx]*cos_array_scale[7-idx%8]/math.sqrt(r2_array[idx]),4)
    #             y_array[idx] = round(r_array[idx]*sin_array_scale[7-int(idx/8)]/math.sqrt(r2_array[idx]),4)
    #         else:
    #             x_array[idx] = round(r_array[idx]*cos_array_scale[idx%8]/math.sqrt(r2_array[idx]),4)
    #             y_array[idx] = round(r_array[idx]*sin_array_scale[int(idx/8)]/math.sqrt(r2_array[idx]),4)

    #     points = np.vstack((np.asarray(x_array),np.asarray(y_array),np.asarray(z_array))).T
    #     pcd = point_cloud(points, tof_msg.header.frame_id)
    #     self.pcd_publisher[tof_position].publish(pcd)
    #     # self.left_head_pcd = point_cloud(points, 'left_head')
    #     # self.left_head_pcd_publisher.publish(self.left_head_pcd)
    #     self.get_logger().info('I heard: "%s"' % tof_msg.header.frame_id + 'timestamp: "%d"' % tof_time)

    def head_tof_pcd_generate(self, msg = HeadTofPayload()):
        global lc,lcm_publish_flag
        z_array = [0]*64
        x_array = [0]*64
        y_array = [0]*64
        r_array = [0]*64
        if len(msg.left_head.data) != 64 or len(msg.right_head.data) != 64:
            self.get_logger().info("Invalid lenth of tof data: {}, {}".format(
                len(msg.left_head.data), len(msg.right_head.data)))
            return
        for idx in range(0,64):
            z_array[idx] =  -msg.left_head.data[63-idx]
            r_array[idx] = -z_array[idx]/math.cos(math.pi*angle_arr[idx]/180)*math.sin(math.pi*angle_arr[idx]/180)
            x_array[idx] = round(r_array[idx]*cos_array_scale[7-idx%8]/math.sqrt(r2_array[idx]),4)
            y_array[idx] = round(r_array[idx]*sin_array_scale[7-int(idx/8)]/math.sqrt(r2_array[idx]),4)
            x_array[idx], y_array[idx], z_array[idx] = np.dot(h_left_R, np.array([x_array[idx], y_array[idx], z_array[idx]])) + h_left_t
        left_points = np.vstack((np.asarray(x_array),np.asarray(y_array),np.asarray(z_array))).T
        for idx in range(0,64):
            z_array[idx] =  -msg.right_head.data[63-idx]
            r_array[idx] = -z_array[idx]/math.cos(math.pi*angle_arr[idx]/180)*math.sin(math.pi*angle_arr[idx]/180)
            x_array[idx] = round(r_array[idx]*cos_array_scale[7-idx%8]/math.sqrt(r2_array[idx]),4)
            y_array[idx] = round(r_array[idx]*sin_array_scale[7-int(idx/8)]/math.sqrt(r2_array[idx]),4)
            x_array[idx], y_array[idx], z_array[idx] = np.dot(h_right_R, np.array([x_array[idx], y_array[idx], z_array[idx]])) + h_right_t
        right_points = np.vstack((np.asarray(x_array),np.asarray(y_array),np.asarray(z_array))).T
        # arrange points
        # points = np.array([[]]*3).T
        # for row in range(0,8):
        #     left_tmp_points = np.array([[]]*3).T
        #     right_tmp_points = np.array([[]]*3).T
        #     for col in range(0,8):
        #         #  Raw:
        #         #  *  *  *  *  *  * 55 63   *  *  *  *  *  *  *  0  
        #         #  *  *  *  *  *  *  * 62   *  *  *  *  *  *  *  1 
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  *  2 
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  *  * 
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  *  * 
        #         #  2  *  *  *  *  *  *  *   *  *  *  *  *  *  *  * 
        #         #  1  *  *  *  *  *  * 57  62  *  *  *  *  *  *  * 
        #         #  0  *  *  *  *  *  * 56  63 55  *  *  *  *  *  7 
        #         #                           
        #         #                        | x
        #         #                   y    |
        #         #                    ————   
        #         # Concatenated:  
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  * 127  
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  *  1 
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  *  2 
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  *  * 
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  *  * 
        #         #  *  *  *  *  *  *  *  *   *  *  *  *  *  *  *  * 
        #         # 16  *  *  *  *  *  *  *   *  *  *  *  *  *  * 31 
        #         #  0  1  2  *  *  *  *  7   8  9  *  *  *  *  * 15               
        #         left_tmp_points = np.concatenate((left_tmp_points, left_points[[col*8+row], :]), axis=0)
        #         right_tmp_points  = np.concatenate((right_tmp_points, right_points[[(8-col)*8-1-row], :]), axis=0)
        #     tmp_points = np.concatenate((left_tmp_points, right_tmp_points), axis=0)
        #     points = np.concatenate((points, tmp_points), axis=0)
        points = np.concatenate((left_points, right_points), axis=0)
        pcd = point_cloud(points, "robot")
        if self.head_pcd_publisher.get_subscription_count() > 0:
            self.head_pcd_publisher.publish(pcd)
        if lcm_publish_flag:
            tof_points_msg=tof_lcmt.tof_lcmt()
            tof_points_msg.tof_id = 0
            tof_points_msg.tof_time = 0 #msg.stamp.nanosec/1000000 + msg.header.stamp.sec * 1000 #待定
            for idx in range(0,64):
                tof_points_msg.tof_data[idx] = (float)(msg.left_head.data[idx])
            self.lock.acquire()
            try:
                lc.publish("tof_points",tof_points_msg.encode())
            finally:
                self.lock.release()
            tof_points_msg.tof_id = 1
            for idx in range(0,64):
                tof_points_msg.tof_data[idx] = (float)(msg.right_head.data[idx])
            self.lock.acquire()
            try:
                lc.publish("tof_points",tof_points_msg.encode())
            finally:
                self.lock.release()
        return

    def rear_tof_pcd_generate(self, msg = RearTofPayload()):
        global lc
        z_array = [0]*64
        x_array = [0]*64
        y_array = [0]*64
        r_array = [0]*64
        if len(msg.left_rear.data) != 64 or len(msg.right_rear.data) != 64:
            self.get_logger().info("Invalid lenth of tof data: {}, {}".format(
                len(msg.left_rear.data), len(msg.right_rear.data)))
            return

        for idx in range(0,64):
            z_array[idx] =  -msg.left_rear.data[63-idx]
            r_array[idx] = -z_array[idx]/math.cos(math.pi*angle_arr[idx]/180)*math.sin(math.pi*angle_arr[idx]/180)
            x_array[idx] = round(r_array[idx]*cos_array_scale[7-idx%8]/math.sqrt(r2_array[idx]),4)
            y_array[idx] = round(r_array[idx]*sin_array_scale[7-int(idx/8)]/math.sqrt(r2_array[idx]),4)
            x_array[idx], y_array[idx], z_array[idx] = np.dot(r_left_R, np.array([x_array[idx], y_array[idx], z_array[idx]])) + r_left_t
        left_points = np.vstack((np.asarray(x_array),np.asarray(y_array),np.asarray(z_array))).T
        for idx in range(0,64):
            z_array[idx] =  -msg.right_rear.data[63-idx]
            r_array[idx] = -z_array[idx]/math.cos(math.pi*angle_arr[idx]/180)*math.sin(math.pi*angle_arr[idx]/180)
            x_array[idx] = round(r_array[idx]*cos_array_scale[7-idx%8]/math.sqrt(r2_array[idx]),4)
            y_array[idx] = round(r_array[idx]*sin_array_scale[7-int(idx/8)]/math.sqrt(r2_array[idx]),4)
            x_array[idx], y_array[idx], z_array[idx] = np.dot(r_right_R, np.array([x_array[idx], y_array[idx], z_array[idx]])) + r_right_t
        right_points = np.vstack((np.asarray(x_array),np.asarray(y_array),np.asarray(z_array))).T
        points = np.concatenate((left_points, right_points), axis=0)
        pcd = point_cloud(points, "robot")
        if self.rear_pcd_publisher.get_subscription_count() > 0:
            self.rear_pcd_publisher.publish(pcd)

        if lcm_publish_flag:
            tof_points_msg=tof_lcmt.tof_lcmt()        
            tof_points_msg.tof_id = 2
            tof_points_msg.tof_time =  0 #msg.stamp.nanosec/1000000 + msg.header.stamp.sec * 1000 #待定
            for idx in range(0,64):
                tof_points_msg.tof_data[idx] = (float)(msg.left_rear.data[idx])
            self.lock.acquire()
            try:
                lc.publish("tof_points",tof_points_msg.encode())
            finally:
                self.lock.release()
            tof_points_msg.tof_id = 3
            for idx in range(0,64):
                tof_points_msg.tof_data[idx] = (float)(msg.right_rear.data[idx])
            self.lock.acquire()
            try:
                lc.publish("tof_points",tof_points_msg.encode())
            finally:
                self.lock.release()
        return

    def head_listener_callback(self, msg):
        # self.tof_pcd_generate(msg.left_head)
        # self.tof_pcd_generate(msg.right_head)
        self.head_tof_pcd_generate(msg)
        
    def rear_listener_callback(self, msg):
        self.rear_tof_pcd_generate(msg)

def point_cloud(points, parent_frame):
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()