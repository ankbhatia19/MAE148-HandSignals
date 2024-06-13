from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_test_pkg',
            executable='camera_test',
            output='screen'),
    ])
