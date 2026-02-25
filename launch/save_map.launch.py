import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_name = LaunchConfiguration('map_name', default='my_map')

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        arguments=[
            '-f', [os.path.expanduser('~'), '/articubot_two_ws/maps/', map_name],
            '--ros-args', '-p', 'use_sim_time:=true'
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='my_map'),
        map_saver,
    ])
