from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/video0',
        description='Port of the camera (e.g., /dev/video0 for USB camera)'
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Frames per second of the camera'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera1',
        description='Name of the camera'
    )

    # Create the node
    camera_node = Node(
        package='thomas_cameras',
        executable='thomas_cameras_node',
        name='thomas_cameras_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'fps': LaunchConfiguration('fps'),
            'camera_name': LaunchConfiguration('camera_name'),
        }]
    )

    return LaunchDescription([
        port_arg,
        fps_arg,
        camera_name_arg,
        camera_node
    ])
