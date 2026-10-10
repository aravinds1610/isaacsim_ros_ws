#!/usr/bin/env python3
"""Bring up MoveIt2 (move_group + RViz) for the Isaac Sim UR5e.

Replaces ur_moveit_config's own ur_moveit.launch.py, which can't be used here
for two reasons (see README.md):

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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_bridge = LaunchConfiguration('launch_bridge')

    # Isaac Sim stamps /joint_states with simulation time (seconds since Play),
    # not wall clock. Without this, move_group compares a sim-time stamp against
    # its own wall-clock now(), sees a ~1.7e9 s gap, and refuses to execute:
    # "Failed to validate trajectory: couldn't receive full current joint state
    # within 1s". Planning tolerates the mismatch (it falls back to last-known
    # joint values); trajectory execution does not. Requires the Action Graph to
    # publish /clock via ROS2 Publish Clock <- Isaac Read Simulation Time.
    # Declared as a launch arg (not hardcoded) so the stack can still be run
    # against a wall-clock source without editing this file.
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    pkg_share = get_package_share_directory('ur5e_isaac_control')

    # M1b: the composed description (arm + Robotiq 2F-85 + grasp_tcp), not the
    # bare arm. ur5e.urdf is still the source of the arm half; this xacro
    # includes it verbatim.
    urdf_path = os.path.join(pkg_share, 'urdf', 'ur5e_gripper.urdf.xacro')

    # SRDF comes from our Setup Assistant output, NOT ur_moveit_config's.
    # The stock UR SRDF tips the ur_manipulator chain at tool0 (the bare flange)
    # and has no disable_collisions entry for the gripper, so with gripper
    # geometry present every state would read as self-colliding and nothing
    # would plan. Ours tips at grasp_tcp and carries the auto-generated
    # collision matrix.
    #
    # Both files declare <robot name="ur5e">; MoveIt rejects the pair if the
    # names differ, which is why ur5e_gripper.urdf.xacro keeps that name.
    #
    # It is a COPY of src/ur5e_gripper_moveit_config/config/ur5e.srdf rather
    # than a reference, because that package exec_depends on this one and
    # depending back would be a cycle. See that file's header for how to
    # refresh it.
    srdf_path = os.path.join(pkg_share, 'config', 'ur5e_gripper.srdf')

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur5e', package_name='ur_moveit_config')
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(Path('config') / 'kinematics.yaml')
        .joint_limits(Path('config') / 'joint_limits.yaml')
        .trajectory_execution(Path('config') / 'moveit_controllers.yaml')
        # Pilz (PTP/LIN) is deterministic: same request, same path, every time.
        # OMPL samples randomly and wanders in an empty scene. OMPL stays the
        # default so callers that don't name a pipeline are unaffected.
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl', 'pilz_industrial_motion_planner'])
        # Pilz's LIN needs Cartesian speed limits; joint accel limits come
        # from joint_limits.yaml above.
        .pilz_cartesian_limits(Path('config') / 'pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )

    # Publishes TF from Isaac Sim's /joint_states + the URDF.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Gives MoveIt the FollowJointTrajectory action server it expects, backed by
    # Isaac Sim's /joint_command topic.
    trajectory_bridge = Node(
        package='ur5e_isaac_control',
        executable='trajectory_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_bridge),
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'publish_robot_description_semantic': True},
            {'use_sim_time': use_sim_time},
            # Before executing, MoveIt checks the trajectory's first waypoint
            # against the robot's actual pose and refuses if they differ by
            # more than this. The 0.01 rad default is tight for a physics sim:
            # the arm settles slightly under gravity after each move, and at
            # velocity scaling 0.5 the residual tracking error was measured at
            # 0.0095 rad -- right on the limit, so Execute aborted
            # intermittently with "start point deviates from current robot
            # state". 0.05 rad (2.9 deg) still catches a genuinely stale plan
            # while tolerating normal settling.
            {'trajectory_execution.allowed_start_tolerance': 0.05},
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
            {'use_sim_time': use_sim_time},
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
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description="Take time from Isaac Sim's /clock instead of the "
                        'wall clock. Required for trajectory execution; the '
                        'Action Graph must publish /clock.'),
        robot_state_publisher,
        trajectory_bridge,
        move_group,
        rviz,
    ])
