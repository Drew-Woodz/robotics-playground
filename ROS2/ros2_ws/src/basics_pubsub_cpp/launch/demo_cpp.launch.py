from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='basics_pubsub_cpp', executable='simple_pub_cpp', output='screen'),
        Node(package='basics_pubsub_cpp', executable='simple_sub_cpp', output='screen'),
    ])
