import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'double_pendulum_arm'
    pkg_path = get_package_share_directory(pkg_name)

    # Process URDF from Xacro
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Path to RViz config
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')

    return LaunchDescription([
        # Robot State Publisher - publishes TF transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config.toxml()}]
        ),
        # Joint State Publisher GUI - sliders for joint control
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        # RViz2 - 3D visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
            output='screen'
        )
    ])
