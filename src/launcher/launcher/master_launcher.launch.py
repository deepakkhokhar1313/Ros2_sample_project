from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_controller',
            executable='arm_controller_node',
            name='arm_controller_node',
            output='screen'
        ),
          Node(
            package='visualizer',
            executable='visualizer_node',
            name='visualizer',
            output='screen'
        )
    ])