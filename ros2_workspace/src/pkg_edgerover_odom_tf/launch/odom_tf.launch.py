from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odom'),

        # launch 层默认把子坐标系设为 base_footprint，
        # 与 chuyun.urdf 根节点一致；传 child_frame:='' 可恢复用消息里的 base_link
        DeclareLaunchArgument('child_frame', default_value='base_footprint'),

        Node(
            package='pkg_edgerover_odom_tf',
            executable='odom_tf_node',
            name='odom_tf',
            # 把 launch 参数 odom_topic 和 child_frame 一起传给节点
            parameters=[{'odom_topic': LaunchConfiguration('odom_topic'),
                         'child_frame': LaunchConfiguration('child_frame')}],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
