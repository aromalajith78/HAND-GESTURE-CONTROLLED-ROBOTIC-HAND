#!/usr/bin/env python3
"""Launch Gazebo Classic simulation, spawn the 4-DOF arm, and activate ros2_control controllers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    world_file = LaunchConfiguration('world')

    default_world = PathJoinSubstitution(
        [FindPackageShare('robot_arm_gazebo'), 'worlds', 'arm_workspace.world']
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('robot_arm_gazebo'), 'config', 'controllers.yaml']
    )
    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare('robot_arm_description'), 'urdf', 'robot_arm.urdf.xacro']
    )

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            urdf_xacro,
            ' ',
            'controllers_file:=',
            controllers_file,
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
        ),
        condition=IfCondition(use_gazebo_gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robotic_arm_4dof',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.750',
        ],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    spawn_jsb_after_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    spawn_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            description='Launch Gazebo client GUI window',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to Gazebo world file',
        ),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_entity,
        spawn_jsb_after_entity,
        spawn_controllers_after_jsb,
    ])
