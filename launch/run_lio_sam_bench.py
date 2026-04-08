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


def _liorf_node(executable, name, parameters, remappings=None):
    return Node(
        package='liorf',
        executable=executable,
        name=name,
        parameters=parameters,
        remappings=remappings or [],
        output='screen'
    )


def generate_launch_description():

    share_dir = get_package_share_directory('liorf')
    parameter_file = LaunchConfiguration('params_file')
    config_override = LaunchConfiguration('config_override')
    enable_rviz = LaunchConfiguration('enable_rviz')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'mapping.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'lio_sam_ouster.yaml'),
        description='FPath to the ROS2 parameters file to use.')
    
    config_override_declare = DeclareLaunchArgument(
        'config_override',
        default_value='/config/override.yaml',
        description='Path to override parameters file (optional).')
    
    rviz_declare = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization (true by default, set to false for headless use)')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='',
        description='Optional override for use_sim_time (true/false). Empty keeps YAML value.')

    def launch_setup(context, *args, **kwargs):
        node_parameters = _build_node_parameters(context, parameter_file, config_override)
        return [
            _liorf_node(
                'liorf_imuPreintegration',
                'liorf_imuPreintegration',
                node_parameters,
                remappings=[('/liorf/mapping/odometry', '/estimated_odom')]
            ),
            _liorf_node(
                'liorf_imageProjection',
                'liorf_imageProjection',
                node_parameters
            ),
            _liorf_node(
                'liorf_mapOptmization',
                'liorf_mapOptmization',
                node_parameters,
                remappings=[('/liorf/mapping/odometry', '/estimated_odom')]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                condition=IfCondition(enable_rviz),
                arguments=['-d', rviz_config_file],
                output='screen'
            )
        ]

    return LaunchDescription([
        params_declare,
        config_override_declare,
        rviz_declare,
        use_sim_time_declare,
        OpaqueFunction(function=launch_setup)
    ])