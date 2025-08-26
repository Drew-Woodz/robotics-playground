from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    model_path = os.path.join(
        get_package_share_directory('sim_gazebo'), 'models', 'box', 'model.sdf'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'demo_box', '-file', model_path, '-x', '0', '-y', '0', '-z', '1'],
        output='screen'
    )

    return LaunchDescription([gazebo, spawn])
