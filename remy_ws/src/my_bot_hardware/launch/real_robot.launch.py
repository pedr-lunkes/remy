from launch import LaunchDescription
from launch_ros.actions import Node
import xacro, os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf = xacro.process_file(
        os.path.join(
            get_package_share_directory('urdf2_description'),
            'urdf', 'urdf2.xacro'
        )
    ).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': urdf},
                os.path.join(
                    get_package_share_directory('my_bot_hardware'),
                    'config', 'diff_drive_controller.yaml'
                )
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster']
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller']
        ),
    ])
