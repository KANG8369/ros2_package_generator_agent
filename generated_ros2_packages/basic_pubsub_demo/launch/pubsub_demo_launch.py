#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch both the talker and listener nodes to demonstrate pub-sub."""

    talker = Node(
        package='basic_pubsub_demo',
        executable='talker_node',
        name='talker',
        output='screen',
    )

    listener = Node(
        package='basic_pubsub_demo',
        executable='listener_node',
        name='listener',
        output='screen',
    )

    return LaunchDescription([
        talker,
        listener,
    ])
