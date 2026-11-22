# 一条命令拉起完整 TF 树：odom→base_footprint（odom_tf）+ URDF 静态/关节 TF（robot_state_publisher + joint_state_publisher）
# 注意：不要与 chuyun_bringup.launch.py 同时运行，其中的 EKF 也会发布 odom→base_footprint
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # chuyun.urdf 的安装路径（第三方包 chuyun_description 提供，只读取不修改）
    urdf_path = PathJoinSubstitution([
        FindPackageShare('chuyun_description'), 'urdf', 'chuyun.urdf'])

    # 用 cat 读出 URDF 文本作为 robot_description 参数；ParameterValue(value_type=str) 防止被当成 YAML 解析
    robot_description = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    return LaunchDescription([
        # 透传给 odom_tf 的子坐标系参数，默认 base_footprint（URDF 根节点）
        DeclareLaunchArgument('child_frame', default_value='base_footprint'),

        # 复用已验证的 odom_tf.launch.py，避免两处重复配置 odom_tf 节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('pkg_edgerover_odom_tf'), 'launch', 'odom_tf.launch.py'])),
            launch_arguments={'child_frame': LaunchConfiguration('child_frame')}.items(),
        ),

        # 按 URDF 广播 base_footprint→base_link→各部件 的静态 TF，以及车轮关节 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # 给 continuous 车轮关节发布 0 位置的 /joint_states，使左右轮 TF 不缺失（无 GUI 版）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
    ])