import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


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

    return LaunchDescription([
        params_declare,
        config_override_declare,
        rviz_declare,
        Node(
            package='liorf',
            executable='liorf_imuPreintegration',
            name='liorf_imuPreintegration',
            parameters=[parameter_file, config_override],
            remappings=[('/liorf/mapping/odometry', '/estimated_odom')],
            output='screen'
        ),
        Node(
            package='liorf',
            executable='liorf_imageProjection',
            name='liorf_imageProjection',
            parameters=[parameter_file, config_override],
            output='screen'
        ),
        Node(
            package='liorf',
            executable='liorf_mapOptmization',
            name='liorf_mapOptmization',
            parameters=[parameter_file, config_override],
            remappings=[('/liorf/mapping/odometry', '/estimated_odom')],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(enable_rviz),
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])