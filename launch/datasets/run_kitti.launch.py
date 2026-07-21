import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_LAUNCH_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if _LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _LAUNCH_DIR)

from rviz_config_resolver import default_rviz_config_path


def generate_launch_description():
    share_dir = get_package_share_directory('liorf')
    launch_dir = os.path.join(share_dir, 'launch')
    default_rviz_config = default_rviz_config_path(share_dir)

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'kitti.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='',
        description='Optional override for use_sim_time (true/false). Empty keeps YAML value.'
    )

    rviz_declare = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Enable RViz (true/false).'
    )

    rviz_config_declare = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config file.'
    )

    include_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'liorf.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'rviz': rviz,
            'rviz_config': rviz_config,
        }.items(),
    )

    return LaunchDescription([
        params_declare,
        rviz_declare,
        rviz_config_declare,
        use_sim_time_declare,
        include_core,
    ])