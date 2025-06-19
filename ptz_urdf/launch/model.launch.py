#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 공유 디렉터리(런치 시점이 아닌 Python 시점에 경로 계산)
    pkg_path = get_package_share_directory('ptz_urdf')
    urdf_path = os.path.join(pkg_path, 'urdf', 'pan_tilt.urdf')
    config_path = os.path.join(pkg_path, 'config', 'controllers.yaml')
    
    declare_model = DeclareLaunchArgument(
        'model',
        default_value=urdf_path,
        description='Absolute path to robot URDF file'
    )

    # 3) 파이썬으로 URDF 파일 내용 읽기
    with open(urdf_path, 'r') as inf:
        robot_description_content = inf.read()

    # 4) 노드 정의
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    # for gui controller 
#    jsp_node = Node(
#        package='joint_state_publisher',
#        executable='joint_state_publisher',
#        name='joint_state_publisher',
#        output='screen',
#        parameters=[
#            {'robot_description': robot_description_content},
#            {'use_gui': True}
#        ]
#    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'pan_tilt.rviz')]
    )
    
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            config_path
        ],
        output='screen'
    )
    
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    spawn_pan = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_joint_controller'],
        output='screen'
    )
    spawn_tilt = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tilt_joint_controller'],
        output='screen'
    )

    return LaunchDescription([
        declare_model,
        rsp_node,
#        jsp_node,
        control_node,
        spawn_jsb,
        spawn_pan,
        spawn_tilt,
        rviz_node,
    ])

