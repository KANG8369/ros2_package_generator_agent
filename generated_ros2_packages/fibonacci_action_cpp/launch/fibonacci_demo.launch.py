from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fibonacci_action_cpp',
            executable='fibonacci_action_server',
            name='fibonacci_action_server',
            output='screen'
        ),
        Node(
            package='fibonacci_action_cpp',
            executable='fibonacci_action_client',
            name='fibonacci_action_client',
            output='screen',
            parameters=[{'order': 10}]
        ),
    ])
