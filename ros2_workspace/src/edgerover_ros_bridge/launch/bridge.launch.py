import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('edgerover_ros_bridge'), 'config', 'bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_cfg),
        Node(
            package='edgerover_ros_bridge',
            executable='ros_bridge',
            name='edgerover_ros_bridge',
            parameters=[LaunchConfiguration('config')],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])