from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch the integer addition server and a demo client.

    The client is launched with example arguments 2 and 3.
    You can also run your own client from the command line:

        ros2 run integer_adder adder_client 5 7
    """

    server_node = Node(
        package='integer_adder',
        executable='adder_server',
        name='adder_server',
        output='screen'
    )

    client_node = Node(
        package='integer_adder',
        executable='adder_client',
        name='adder_client_demo',
        output='screen',
        arguments=['2', '3']
    )

    return LaunchDescription([
        server_node,
        client_node,
    ])
