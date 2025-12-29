from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default_sensors.yaml',
        description='YAML file with sensor configuration (relative to dummy_sensor_suite/config).'
    )

    config_file = LaunchConfiguration('config_file')
    pkg_share = FindPackageShare('dummy_sensor_suite')

    def get_config_path(context):
        pkg_path = pkg_share.perform(context)
        rel = config_file.perform(context)
        return os.path.join(pkg_path, 'config', rel)

    dummy_node = Node(
        package='dummy_sensor_suite',
        executable='dummy_sensors_node',
        name='dummy_sensors',
        output='screen',
        parameters=[get_config_path]
    )

    # Basic RViz2 configuration to visualize topics (user can load their own config)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share.find('dummy_sensor_suite'), 'config', 'dummy_sensors.rviz')],
        condition=None,
    )

    return LaunchDescription([
        config_arg,
        GroupAction([
            dummy_node,
            rviz_node
        ])
    ])
