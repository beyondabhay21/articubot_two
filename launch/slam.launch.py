import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share_dir = get_package_share_directory('articubot_two_description')
    world_file = os.path.join(share_dir, 'worlds', 'articubot_world.world')
    xacro_file = os.path.join(share_dir, 'urdf', 'articubot_two.xacro')
    robot_urdf = xacro.process_file(xacro_file).toxml()
    slam_config = os.path.join(share_dir, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(share_dir, 'config', 'slam.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_urdf,
            'use_sim_time': True,
        }],
        output='screen',
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'false',
            'verbose': 'false',
            'world': world_file,
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ])
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'articubot_two',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen',
        )]
    )

    joint_state_broadcaster = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        )]
    )

    diff_drive_controller = TimerAction(
        period=12.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'diff_drive_controller',
                '--controller-manager', '/controller_manager',
            ],
            output='screen',
        )]
    )

    # SLAM toolbox node
    slam = TimerAction(
        period=14.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_config,
                {'use_sim_time': True}
            ],
            output='screen',
        )]
    )

    rviz = TimerAction(
        period=10.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        joint_state_broadcaster,
        diff_drive_controller,
        slam,
        rviz,
    ])
