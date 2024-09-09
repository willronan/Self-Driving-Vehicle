from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab4',
            node_name='motor',
            node_executable='motor'
        ),
        Node(
            package='lab4',
            node_executable='gamepad',
            node_name='gamepad'
        ),
        Node(
            package='lab4',
            node_executable='lidar',
            node_name='lidar'
        )
    ])