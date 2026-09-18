#!/usr/bin/env python3
"""Bring up the whole pick stack: MoveIt + perception + the orchestrator (M4).

One command for what was three terminals. Composes rather than duplicates —
moveit.launch.py is *included*, not copied, so the MoveItConfigsBuilder setup
and the rviz snap-env workaround stay in exactly one place.

    Isaac  --/rgb,/camera_info-->  cube_pose_publisher  --/cube_pose-->  |
                                                                        |
      pick_orchestrator --MoveGroup action--> move_group                |
                                                   |                    |
      Isaac  <--/joint_command--  trajectory_bridge <--FollowJointTraj--+

Isaac Sim must be loaded and *playing* before this runs: it supplies
/joint_states, /clock, /rgb and /camera_info, and consumes /joint_command.

WHY auto_start DEFAULTS TO FALSE HERE, WHILE THE NODE'S OWN DEFAULT IS TRUE:
pick_orchestrator begins its sequence 2 s (sim time) after it comes up. Starting
everything at once means the arm moves before you have had a chance to confirm
RViz looks sane and that /cube_pose is actually flowing — and move_group is not
always ready inside 2 s, so it is a genuine startup race. Bring the stack up
idle, check it, then fire the pick deliberately:

    ros2 service call /pick_orchestrator/pick_once std_srvs/srv/Trigger

Pass auto_start:=true once you trust the sequence and want a one-command demo.

Usage:
    ros2 launch ur5e_isaac_control pick_demo.launch.py
    ros2 launch ur5e_isaac_control pick_demo.launch.py auto_start:=true
    ros2 launch ur5e_isaac_control pick_demo.launch.py launch_rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_perception = LaunchConfiguration('launch_perception')
    launch_orchestrator = LaunchConfiguration('launch_orchestrator')

    # Same reasoning as moveit.launch.py: Isaac stamps everything with sim time,
    # so every node in the stack has to read the same clock or trajectory
    # execution fails on a ~1.7e9 s stamp mismatch. Declared as an arg rather
    # than hardcoded so the stack can still run against a wall-clock source.
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    auto_start = ParameterValue(
        LaunchConfiguration('auto_start'), value_type=bool)

    # move_group + robot_state_publisher + trajectory_bridge + rviz.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ur5e_isaac_control'),
             'launch', 'moveit.launch.py'])),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'launch_rviz': launch_rviz,
            # The bridge is what MoveIt executes onto; never split it from
            # move_group, so it is deliberately not exposed as an arg here.
            'launch_bridge': 'true',
        }.items(),
    )

    # /rgb + /camera_info -> /cube_pose in base_link. All the scene-dependent
    # numbers (extrinsic, tag_size, tag_z_base) are node parameters with
    # defaults measured 2026-08-11 — see docs/PLAN.md's scene reference table.
    # Override them here if the scene moves, rather than editing the node.
    cube_pose_publisher = Node(
        package='ur5e_isaac_control',
        executable='cube_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_perception),
    )

    # The M4 state machine. Runs with an EMPTY planning scene by design — see
    # the node's docstring for why M3 is skipped rather than merely deferred.
    pick_orchestrator = Node(
        package='ur5e_isaac_control',
        executable='pick_orchestrator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_start': auto_start,
        }],
        condition=IfCondition(launch_orchestrator),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="Take time from Isaac Sim's /clock instead of the "
                        'wall clock. Required for trajectory execution; the '
                        'Action Graph must publish /clock.'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='Run the pick sequence automatically 2 s after '
                        'startup. False brings the stack up idle; fire it with '
                        '`ros2 service call /pick_orchestrator/pick_once '
                        'std_srvs/srv/Trigger`.'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Launch RViz with the MoveIt Motion Planning panel.'),
        DeclareLaunchArgument(
            'launch_perception', default_value='true',
            description='Launch cube_pose_publisher. False if you want to run '
                        'it separately, or to replay /cube_pose from a bag.'),
        DeclareLaunchArgument(
            'launch_orchestrator', default_value='true',
            description='Launch pick_orchestrator. False gives you MoveIt + '
                        'perception only, for driving the arm by hand from '
                        "RViz's Motion Planning panel."),
        moveit,
        cube_pose_publisher,
        pick_orchestrator,
    ])
