from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('thomas_joystick_controller'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='thomas_driver',
            executable='thomas_driver_node',
            name='thomas_driver_node'
        ),
        Node(
            package='thomas_joystick_controller',
            executable='thomas_joystick_controller_node',
            name='thomas_joystick_controller_node',
            parameters=[config_file]
        ),
    ])
