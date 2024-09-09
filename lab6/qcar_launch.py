from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab5_groupX',
            node_name='motor',
            node_executable='motor'
        ),
        Node(
            package='lab5_groupX',
            node_executable='steering',
            node_name='steering'
        ),
        Node(
            package='lab5_groupX',
            node_executable='sign_detection',
            node_name='sign_detection'
        ),
        Node(
            package='lab5_groupX',
            node_executable='lidar',
            node_name='lidar'
        )
    ])