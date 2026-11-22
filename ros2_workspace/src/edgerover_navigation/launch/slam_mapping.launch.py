# 在线异步建图：复用 slam_toolbox 官方 online_async_launch.py，参数文件默认取仓库 config/slam_toolbox.yaml
# 前提：edgerover-uros-agent / edgerover-tf 两个服务已运行（/scan、/odom、完整 TF 树）
import os
from launch                             import LaunchDescription
from launch.actions                     import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions           import FindPackageShare


def generate_launch_description():
    # 与 supervisor 一致按 $HOME/edge_rover 定位，参数改完无需重新 colcon build
    default_params = os.path.join(os.path.expanduser('~'), 'edge_rover', 'config', 'slam_toolbox.yaml')

    return LaunchDescription([

        # 可用 slam_params_file:=xxx.yaml 临时换参数做对比
        DeclareLaunchArgument('slam_params_file', default_value=default_params),

        # 实车运行，不用仿真时钟
        DeclareLaunchArgument('use_sim_time', default_value='False'),

        # 用 slam_toolbox 自带的 launch，避免手抄节点名/重映射
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('slam_toolbox'), 
                                                                'launch', 
                                                                'online_async_launch.py'])),
            launch_arguments=
            {
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time':     LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])