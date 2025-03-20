# imu_filters/imu_filters/launch/filters.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filters',
            executable='filter_node',
            name='imu_filter_node',
            parameters=[{'filter_type': 'median'}],  # Change to 'mean' or 'lowpass' if needed
        )
    ])
