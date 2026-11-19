#!/usr/bin/env python3
"""Complete system startup: Gazebo simulation, ros2_control, IK motion controller, and RViz2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    launch_gui = LaunchConfiguration('launch_gui')
    launch_hand_control = LaunchConfiguration('launch_hand_control')

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('robot_arm_bringup'), 'rviz', 'robot_arm.rviz']
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robot_arm_gazebo'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'use_gazebo_gui': use_gazebo_gui}.items(),
    )

    arm_motion_controller = Node(
        package='robot_arm_control',
        executable='arm_motion_controller',
        name='arm_motion_controller',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    gui_node = Node(
        package='robot_arm_control',
        executable='robot_arm_gui',
        name='robot_arm_gui',
        output='screen',
        condition=IfCondition(launch_gui),
    )

    hand_control_node = Node(
        package='robot_arm_control',
        executable='hand_control',
        name='camera_hand_control',
        output='screen',
        condition=IfCondition(launch_hand_control),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 visualization synchronized with simulation',
        ),
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            description='Start Gazebo Classic GUI client',
        ),
        DeclareLaunchArgument(
            'launch_gui',
            default_value='false',
            description='Optionally auto-launch the Tkinter arm control GUI',
        ),
        DeclareLaunchArgument(
            'launch_hand_control',
            default_value='false',
            description='Optionally auto-launch webcam hand-gesture teleoperation',
        ),
        gazebo_launch,
        arm_motion_controller,
        rviz2,
        gui_node,
        hand_control_node,
    ])
