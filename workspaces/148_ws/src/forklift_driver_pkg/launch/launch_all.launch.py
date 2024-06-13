from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ucsd_robocar_lane_detection2_pkg'),
        'config',
        'ros_racer_calibration.yaml')
    return LaunchDescription([
        Node(
            package='forklift_driver_pkg',
            executable='forklift_driver',
            output='screen'),
        Node(
            package='servo_control_pkg',
            executable='servo_control',
            output='screen'),
        #Node(
        #    package='camera_pkg',
        #    executable='camera_pos',
        #    output='screen'),
        Node(
            package='ucsd_robocar_actuator2_pkg',
            executable='vesc_twist_node',
            output='screen',
            parameters=[config]),
    ])
