from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usv_nav',
            executable='usv_nav',
            name='usv_nav_node',
            output='screen',
            emulate_tty=True,
        )
    ])
