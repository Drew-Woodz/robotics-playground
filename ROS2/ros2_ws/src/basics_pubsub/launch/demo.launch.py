from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='basics_pubsub', executable='simple_pub', output='screen'),
        Node(package='basics_pubsub', executable='simple_sub', output='screen'),
    ])
