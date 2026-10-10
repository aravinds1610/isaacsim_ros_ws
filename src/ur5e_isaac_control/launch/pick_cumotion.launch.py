#!/usr/bin/env python3
"""Host half of the cuMotion pick: perception + the orchestrator.

The planning half runs in the Isaac ROS container and must be up FIRST:

    isaac-ros activate
    ros2 launch /workspaces/isaac_ros-dev/launch/ur_isaac_sim.launch.py ur_type:=ur5e

That is NVIDIA's ur_isaac_sim.launch.py (one fix: XRDF follows ur_type) --
move_group with cuMotion as the planner, ros2_control's
joint_trajectory_controller, and topic_based_ros2_control talking to Isaac.

    Isaac --/rgb,/camera_info--> cube_pose_publisher --/cube_pose--> |
                                                                     |
      pick_orchestrator --/compute_ik, MoveGroup action--> move_group + cuMotion  (container)
                                                               |
      Isaac <--/isaac_joint_commands-- joint_trajectory_controller  (container)
      Isaac <--/isaac_joint_commands-- pick_orchestrator (gripper only)

Differences from pick_demo.launch.py (the Pilz/trajectory_bridge path), all
orchestrator parameters:
  - tool frame is tool0, NVIDIA's bare-arm model has no grasp_tcp; the
    fingertips are tcp_offset_z = 0.11181 m beyond it.
  - every move is planned by cuMotion; 'PTP' and 'LIN' both map to it.
  - ik_then_joint_goal: IK from the current state via /compute_ik, then a joint
    goal. cuMotion's own pose-goal IK flipped the elbow on the way to the bin.
  - the gripper is commanded on Isaac's renamed topic, /isaac_joint_commands.

Isaac Sim must be playing. After a Stop/Play, restart BOTH halves (sim time
resets to 0 and the controllers and move_group keep stale timing).

auto_start defaults to false for the same reason as pick_demo.launch.py; fire
the pick with:

    ros2 service call /pick_orchestrator/pick_once std_srvs/srv/Trigger

Usage:
    ros2 launch ur5e_isaac_control pick_cumotion.launch.py
    ros2 launch ur5e_isaac_control pick_cumotion.launch.py auto_start:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    launch_perception = LaunchConfiguration('launch_perception')
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)
    auto_start = ParameterValue(
        LaunchConfiguration('auto_start'), value_type=bool)

    cube_pose_publisher = Node(
        package='ur5e_isaac_control',
        executable='cube_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_perception),
    )

    pick_orchestrator = Node(
        package='ur5e_isaac_control',
        executable='pick_orchestrator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_start': auto_start,
            'pipeline_id': 'isaac_ros_cumotion',
            'ptp_planner_id': 'cuMotion',
            'lin_planner_id': 'cuMotion',
            'tcp_link': 'tool0',
            'tcp_offset_z': 0.11181,
            'ik_then_joint_goal': True,
            'command_topic': '/isaac_joint_commands',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="Take time from Isaac Sim's /clock instead of the "
                        'wall clock. Required for trajectory execution.'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='Run the pick sequence automatically 2 s after '
                        'startup. False brings it up idle; fire it with '
                        '`ros2 service call /pick_orchestrator/pick_once '
                        'std_srvs/srv/Trigger`.'),
        DeclareLaunchArgument(
            'launch_perception', default_value='true',
            description='Launch cube_pose_publisher. False to run it '
                        'separately or replay /cube_pose from a bag.'),
        cube_pose_publisher,
        pick_orchestrator,
    ])
