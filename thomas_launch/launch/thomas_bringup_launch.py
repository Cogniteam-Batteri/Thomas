from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare common launch arguments
    port1_arg = DeclareLaunchArgument(
        'port1',
        default_value='/dev/video0',
        description='Port of the first camera'
    )

    port2_arg = DeclareLaunchArgument(
        'port2',
        default_value='/dev/video2',
        description='Port of the second camera'
    )

    port3_arg = DeclareLaunchArgument(
        'port3',
        default_value='/dev/video4',
        description='Port of the third camera'
    )

    port4_arg = DeclareLaunchArgument(
        'port4',
        default_value='/dev/video6',
        description='Port of the fourth camera'  # Corrected description
    )

    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='15',
        description='Frames per second for all cameras'
    )

    # Define nodes for each camera with unique parameters
    camera1_node = Node(
        package='thomas_cameras',
        executable='thomas_cameras_node',
        name='camera1_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port1'),
            'fps': LaunchConfiguration('fps'),
            'camera_name': 'camera1',
        }]
    )

    camera2_node = Node(
        package='thomas_cameras',
        executable='thomas_cameras_node',
        name='camera2_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port2'),
            'fps': LaunchConfiguration('fps'),
            'camera_name': 'camera2',
        }]
    )

    camera3_node = Node(
        package='thomas_cameras',
        executable='thomas_cameras_node',
        name='camera3_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port3'),
            'fps': LaunchConfiguration('fps'),
            'camera_name': 'camera3',
        }]
    )

    camera4_node = Node(
        package='thomas_cameras',
        executable='thomas_cameras_node',
        name='camera4_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port4'),
            'fps': LaunchConfiguration('fps'),
            'camera_name': 'camera4',
        }]
    )

    return LaunchDescription([
        port1_arg,
        port2_arg,
        port3_arg,
        port4_arg,
        fps_arg,
        camera1_node,  
        camera2_node,
        camera3_node,
        camera4_node
    ])
