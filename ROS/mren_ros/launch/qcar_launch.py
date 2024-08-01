from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mren_ros',
            node_name='driver',
            node_executable='driver'
        ),
        Node(
            package='mren_ros',
            node_executable='lane_detector',
            node_name='lane_detector'
        ),
        Node(
            package='mren_ros',
            node_executable='sign_detector',
            node_name='sign_detector'
        ),
        Node(
            package='mren_ros',
            node_executable='lidar',
            node_name='lidar'
        )
    ])