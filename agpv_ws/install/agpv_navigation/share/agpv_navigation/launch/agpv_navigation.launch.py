from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agpv_navigation',
            executable='patrol_vehicle',
            name='patrol_vehicle',
            output='screen',
        ),
        Node(
            package='agpv_navigation',
            executable='alert_handler',
            name='alert_handler',
            output='screen',
        ),
        Node(
            package='agpv_navigation',
            executable='visual_detector',
            name='visual_detector',
            output='screen',
        ),
    ])
