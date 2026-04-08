import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('liorf')

    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_rviz = LaunchConfiguration('rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'lio_sam_ouster.yaml'),
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

    return LaunchDescription([
        params_declare,
        rviz_config_declare,
        enable_rviz_declare,
        Node(
            package='liorf',
            executable='liorf_imuPreintegration',
            name='liorf_imuPreintegration',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='liorf',
            executable='liorf_imageProjection',
            name='liorf_imageProjection',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='liorf',
            executable='liorf_mapOptmization',
            name='liorf_mapOptmization',
            parameters=[params_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(enable_rviz),
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
