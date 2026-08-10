#!/usr/bin/env python3
"""Bring up MoveIt2 (move_group + RViz) for the Isaac Sim UR5e.

Replaces ur_moveit_config's own ur_moveit.launch.py, which can't be used here
for two reasons (see docs/EXPLAINED.md §3):

1. It launches ur_robot_driver's ``wait_for_robot_description`` and gates
   move_group/RViz behind that node's exit. ur_robot_driver isn't installed —
   it exists to talk to physical UR hardware over the network, which we don't
   want in the loop.
2. Its MoveItConfigsBuilder sets only ``robot_description_semantic`` — it never
   loads a URDF, assuming the real-robot driver already published one.

So we run robot_state_publisher with *our* URDF and parameterise move_group
from ur_moveit_config's SRDF/kinematics/OMPL/limits YAML, which are all plain
files needing no xacro args. 100% of the config reuse, none of the driver.

Isaac Sim supplies /joint_states over the ROS2 bridge and consumes
/joint_command, so it must be loaded and *playing* before this is launched.

Usage:
    ros2 launch ur5e_isaac_control moveit.launch.py
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_bridge = LaunchConfiguration('launch_bridge')

    pkg_share = get_package_share_directory('ur5e_isaac_control')
    urdf_path = os.path.join(pkg_share, 'urdf', 'ur5e.urdf')

    # The SRDF's <robot name="$(arg name)"> must match the URDF's robot name,
    # which is "ur5e" — MoveIt rejects the pair otherwise.
    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur5e', package_name='ur_moveit_config')
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(
            Path('srdf') / 'ur.srdf.xacro', {'name': 'ur5e'})
        .robot_description_kinematics(Path('config') / 'kinematics.yaml')
        .joint_limits(Path('config') / 'joint_limits.yaml')
        .trajectory_execution(Path('config') / 'moveit_controllers.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl', pipelines=['ompl'])
        .to_moveit_configs()
    )

    # Publishes TF from Isaac Sim's /joint_states + the URDF.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # Gives MoveIt the FollowJointTrajectory action server it expects, backed by
    # Isaac Sim's /joint_command topic.
    trajectory_bridge = Node(
        package='ur5e_isaac_control',
        executable='trajectory_bridge',
        output='screen',
        condition=IfCondition(launch_bridge),
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'publish_robot_description_semantic': True},
        ],
    )

    rviz_env = dict(os.environ)

    # When this launch file runs from a shell that inherited VSCode's Snap
    # environment, GTK_PATH/LOCPATH/XDG_DATA_DIRS/etc. point inside
    # /snap/code/.../usr, which drags in that snap's bundled libpthread.so.0
    # ahead of the system one. rviz2 (Qt, dynamically loading GTK/GDK theme
    # and locale plugins) then dies at startup with a GLIBC_PRIVATE symbol
    # lookup error. Strip the snap-injected GUI vars just for this process --
    # every other node here is headless and unaffected.
    for var in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GDK_PIXBUF_MODULEDIR',
                'GDK_PIXBUF_MODULE_FILE', 'GIO_MODULE_DIR',
                'GIO_LAUNCHED_DESKTOP_FILE', 'GIO_LAUNCHED_DESKTOP_FILE_PID',
                'GSETTINGS_SCHEMA_DIR', 'GTK_IM_MODULE_FILE', 'LOCPATH',
                'XDG_DATA_DIRS', 'XDG_DATA_HOME', 'XDG_CONFIG_DIRS'):
        rviz_env.pop(var, None)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        condition=IfCondition(launch_rviz),
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('ur_moveit_config'), 'config', 'moveit.rviz'])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        env=rviz_env,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz with the MoveIt Motion Planning panel.'),
        DeclareLaunchArgument(
            'launch_bridge', default_value='true',
            description='Launch the FollowJointTrajectory -> /joint_command '
                        'bridge that lets MoveIt execute onto Isaac Sim.'),
        robot_state_publisher,
        trajectory_bridge,
        move_group,
        rviz,
    ])
