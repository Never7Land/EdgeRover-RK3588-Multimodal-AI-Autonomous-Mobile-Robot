# Nav2 导航：复用 nav2_bringup 官方 bringup_launch.py（slam=False → AMCL 定位 + 全局规划 + DWB 控制）
# 前提：edgerover-uros-agent / edgerover-tf 两个服务已运行（/scan、/odom、完整 TF 树）
# 不要与 slam_mapping.launch.py 同时运行：两者都发 map→odom（地图到里程计的校正变换）
import os
from launch                             import LaunchDescription
from launch.actions                     import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions           import FindPackageShare


def generate_launch_description():
    # 与 supervisor 一致按 $HOME/edge_rover 定位，参数改完无需重新 colcon build
    default_params = os.path.join(os.path.expanduser('~'), 'edge_rover', 'config', 'nav2_params.yaml')

    return LaunchDescription([

        # 参数文件；可用 params_file:=xxx.yaml 临时换一套做对比
        DeclareLaunchArgument('params_file', default_value=default_params),

        # 地图 yaml 必须由命令行 map:= 显式给出，不设默认值，避免误用 maps/ 下的旧地图
        DeclareLaunchArgument('map', description='地图 yaml 的绝对路径，必填'),

        # 实车运行，不用仿真时钟
        DeclareLaunchArgument('use_sim_time', default_value='False'),

        # 每个 Nav2 节点单独一个进程：出问题一眼看出谁挂了；跑通后可改 true 省内存
        DeclareLaunchArgument('use_composition', default_value='False'),

        # 写成 'True'，与 nav2_bringup（Nav2 官方启动包）里 'True'/'False' 的写法保持一致
        DeclareLaunchArgument('use_respawn', default_value='True'),

        # 用 nav2_bringup 自带的 launch，避免手抄十几个节点和 lifecycle 的依赖顺序
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('nav2_bringup'),
                                                                'launch',
                                                                'bringup_launch.py'])),
            launch_arguments=
            {
                'slam':            'False',         # 不建图，map→odom 交给 AMCL；建图走 slam_mapping.launch.py
                'map':             LaunchConfiguration('map'),
                'params_file':     LaunchConfiguration('params_file'),
                'use_sim_time':    LaunchConfiguration('use_sim_time'),
                'autostart':       'true',          # 自动 configure + activate 全部 lifecycle 节点
                'use_composition': LaunchConfiguration('use_composition'),
                'use_respawn':     LaunchConfiguration('use_respawn'),
            }.items(),
        ),
    ])