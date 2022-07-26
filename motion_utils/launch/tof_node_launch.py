from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_utils',
            executable='head_tof_pcl_publisher.py',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.259', '0.03', '0.102', '0', '-0.266', '0.296', 'robot', 'left_head']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.259', '-0.03', '0.102', '0', '-0.266', '-0.296', 'robot','right_head']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.021', '0.042', '-0.051', '0', '0', '0.296', 'robot', 'left_rear']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.021', '-0.042', '-0.051', '0', '0', '-0.296', 'robot', 'right_rear']
        )
    ])