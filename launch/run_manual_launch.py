from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ocean_manual',
            executable='run_manual',
            name='manual_control_node',
            output='screen'
        )
    ])