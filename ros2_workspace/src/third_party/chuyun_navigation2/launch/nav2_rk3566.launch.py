"""Nav2 + AMCL on RK3528, using the RK3566 robot parameter profiles."""
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    own = get_package_share_directory('chuyun_navigation2')
    bringup = get_package_share_directory('nav2_bringup')
    # Fail instead of silently falling back to an apt-installed Nav2 package.
    source_install = '/root/nav2_ws/install'
    for package in ('nav2_bringup', 'nav2_amcl', 'nav2_controller',
                    'nav2_costmap_2d', 'nav2_planner', 'nav2_util', 'nav2_msgs',
                    'nav2_lifecycle_manager', 'nav2_bt_navigator', 'dwb_core'):
        prefix = os.path.realpath(get_package_prefix(package))
        if os.path.commonpath([source_install, prefix]) != source_install:
            raise RuntimeError('Source Nav2 required for ' + package +
                               '; source /root/nav2_ws/install/local_setup.bash')
    # These source binaries were built against TF2 0.25.22.
    # Keep the matching TF2 runtime local to this launch.
    compat_lib = os.path.join(source_install, 'tf2_compat_0_25_22', 'lib')
    for library in ('libtf2.so', 'libtf2_ros.so'):
        if not os.path.isfile(os.path.join(compat_lib, library)):
            raise RuntimeError('Required Nav2 TF2 install library is missing: ' + os.path.join(compat_lib, library))
    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', compat_lib + os.pathsep + os.environ.get('LD_LIBRARY_PATH', '')),
        DeclareLaunchArgument('map', default_value=os.path.join(own, 'maps', 'room.yaml')),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(own, 'config', 'nav2_rk3566_diff.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'use_composition': PythonExpression([
                    "'", LaunchConfiguration('use_composition'), "'.lower() in ['true', '1']"]),
                'log_level': LaunchConfiguration('log_level'),
                'slam': 'False',
                'use_namespace': 'False',
            }.items()),
        Node(package='chuyun_navigation2', executable='nav2_web_adapter',
             name='nav2_web_adapter', output='screen'),
    ])
