#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    paused       = LaunchConfiguration('paused', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui          = LaunchConfiguration('gui', default='true')
    headless     = LaunchConfiguration('headless', default='false')
    debug        = LaunchConfiguration('debug', default='false')

    # Paths
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    ex_urdf_pkg    = get_package_share_directory('ptz_urdf')
    gazebo_launch    = os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')

    # Xacro command to generate URDF
    robot_description_content = Command([
        'xacro ', 
        os.path.join(ex_urdf_pkg, 'urdf', 'pan_tilt.xacro'),
        ' paused:=', paused,
        ' use_sim_time:=', use_sim_time
    ])

    # Include Gazebo empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'paused': paused,
            'use_sim_time': use_sim_time,
            'gui': gui,
            'headless': headless,
            'debug': debug,
            'world':        os.path.join(gazebo_ros_pkg, 'worlds', 'empty.world'),

        }.items()
    )

    # Spawn robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'urdf_pan_tilt',
        ],
        output='screen'
    )

    return LaunchDescription([
        # Declare args
        DeclareLaunchArgument('paused',       default_value='false', description='Start paused?'),
        DeclareLaunchArgument('use_sim_time', default_value='true',  description='Use simulation time'),
        DeclareLaunchArgument('gui',          default_value='true',  description='Enable gazebo GUI'),
        DeclareLaunchArgument('headless',     default_value='false', description='Run gzserver only'),
        DeclareLaunchArgument('debug',        default_value='false', description='Enable debug logging'),

        # Load robot_description parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),

        gazebo,
        spawn_entity,
    ])

