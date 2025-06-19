#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ptz_urdf')

    # 1) Gazebo 포함 (ros2버전 파일)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo_pan_tilt.launch.py')
        )
    )

    # 2) 컨트롤러 파라미터 로드
    control_yaml = PathJoinSubstitution([pkg_share, 'config', 'pan_tilt_control.yaml'])
    load_controllers = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'load', '/urdf_pan_tilt/controller_manager',
            control_yaml
        ],
        output='screen'
    )

    # 3) 컨트롤러 스포너 노드들
    spawner_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        namespace='urdf_pan_tilt',
        arguments=['joint_state_controller', '--controller-manager', '/urdf_pan_tilt/controller_manager'],
        output='screen'
    )
    spawner_j1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='urdf_pan_tilt',
        arguments=['joint1_position_controller', '--controller-manager', '/ex_urdf_pan_tilt/controller_manager'],
        output='screen'
    )
    spawner_j2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace='urdf_pan_tilt',
        arguments=['joint2_position_controller', '--controller-manager', '/urdf_pan_tilt/controller_manager'],
        output='screen'
    )

    # 4) robot_state_publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',  # global namespace
        output='screen',
        remappings=[('/joint_states', '/urdf_pan_tilt/joint_states')]
    )

    return LaunchDescription([
        gazebo_launch,
        load_controllers,
        spawner_joint_state,
        spawner_j1,
        spawner_j2,
        rsp,
    ])

