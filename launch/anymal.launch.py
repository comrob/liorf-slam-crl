import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_optional_bool(value: str):
    normalized = value.strip().lower()
    if normalized == '':
        return None
    if normalized in ('true', '1', 'yes', 'on'):
        return True
    if normalized in ('false', '0', 'no', 'off'):
        return False
    raise RuntimeError("Invalid 'use_sim_time' value. Use true/false (or leave empty).")


def _build_node_parameters(context, *parameter_sources):
    parameters = list(parameter_sources)
    use_sim_time_value = LaunchConfiguration('use_sim_time').perform(context)
    parsed = _parse_optional_bool(use_sim_time_value)
    if parsed is not None:
        parameters.append({'use_sim_time': parsed})
    return parameters


def generate_launch_description():
    share_dir = get_package_share_directory('liorf')

    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'anymal.yaml'),
        description='Path to the ROS 2 parameters file.'
    )

    rviz_config_declare = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(share_dir, 'rviz', 'mapping.rviz'),
        description='Path to RViz config file.'
    )

    enable_rviz_declare = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Enable RViz (true/false).'
    )

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='',
        description='Optional override for use_sim_time (true/false). Empty keeps YAML value.'
    )

    def launch_setup(context, *args, **kwargs):
        node_parameters = _build_node_parameters(context, params_file)

        return [
            Node(
                package='liorf',
                executable='liorf_imuPreintegration',
                name='liorf_imuPreintegration',
                parameters=node_parameters,
                output='screen'
            ),
            Node(
                package='liorf',
                executable='liorf_imageProjection',
                name='liorf_imageProjection',
                parameters=node_parameters,
                output='screen'
            ),
            Node(
                package='liorf',
                executable='liorf_mapOptmization',
                name='liorf_mapOptmization',
                parameters=node_parameters,
                output='screen',
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                condition=IfCondition(enable_rviz),
                arguments=['-d', rviz_config],
                output='screen'
            )
        ]

    return LaunchDescription([
        params_declare,
        rviz_config_declare,
        enable_rviz_declare,
        use_sim_time_declare,
        OpaqueFunction(function=launch_setup)
    ])
