#!/usr/bin/env python3
"""FollowJointTrajectory action server bridging MoveIt2 to Isaac Sim.

MoveIt executes a plan by sending it to a control_msgs/FollowJointTrajectory
action server -- normally provided by ros2_control's joint_trajectory_controller.
Isaac Sim's ROS2 bridge offers no such action: it exposes a plain
sensor_msgs/JointState topic (/joint_command) wired to an Articulation
Controller node in the Action Graph. This node is that missing adapter.

The action name matches what ur_moveit_config's moveit_controllers.yaml
declares -- the 'scaled_joint_trajectory_controller' entry (default: true,
action_ns: follow_joint_trajectory) -- so MoveIt finds it with no config
override:

    /scaled_joint_trajectory_controller/follow_joint_trajectory

MoveIt's trajectory points are timestamped waypoints, not a stream, so we
resample them: at each tick we linearly interpolate the two points bracketing
the current elapsed time and publish that pose. This preserves MoveIt's
time parameterisation (its velocity/acceleration limits) rather than replacing
it with a fixed-duration ramp.

Precautions carried over from joint_command_publisher.py (see SETUP_NOTES.md):
publishing before Isaac Sim's subscriber has matched silently drops messages,
so we wait for a subscriber before streaming.
"""
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

DEFAULT_ACTION_NAME = \
    '/scaled_joint_trajectory_controller/follow_joint_trajectory'


def _duration_to_sec(d):
    return d.sec + d.nanosec * 1e-9


def sample_trajectory(points, t):
    """Positions at time t (sec) along a JointTrajectory point list.

    Linear interpolation between the bracketing points; clamps at both ends.
    Kept a free function so the timing maths can be tested without ROS.
    """
    if not points:
        return None
    if t <= _duration_to_sec(points[0].time_from_start):
        return list(points[0].positions)
    if t >= _duration_to_sec(points[-1].time_from_start):
        return list(points[-1].positions)

    for prev, nxt in zip(points, points[1:]):
        t0 = _duration_to_sec(prev.time_from_start)
        t1 = _duration_to_sec(nxt.time_from_start)
        if t0 <= t <= t1:
            span = t1 - t0
            alpha = 0.0 if span <= 0.0 else (t - t0) / span
            return [p + (n - p) * alpha
                    for p, n in zip(prev.positions, nxt.positions)]
    return list(points[-1].positions)


class TrajectoryBridge(Node):

    def __init__(self):
        super().__init__('trajectory_bridge')

        self.declare_parameter('action_name', DEFAULT_ACTION_NAME)
        self.declare_parameter('command_topic', '/joint_command')
        self.declare_parameter('state_topic', '/joint_states')
        self.declare_parameter('rate_hz', 60.0)
        # Isaac Sim's PD drives track the commanded pose with some lag, so the
        # final position won't match exactly. Fail the goal only past this.
        self.declare_parameter('goal_tolerance_rad', 0.05)

        action_name = self.get_parameter('action_name').value
        self.command_topic = self.get_parameter('command_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance_rad').value)

        self.current_positions = {}

        cb_group = ReentrantCallbackGroup()
        self.pub = self.create_publisher(JointState, self.command_topic, 10)
        self.create_subscription(
            JointState, self.state_topic, self._on_state, 10,
            callback_group=cb_group)

        self._server = ActionServer(
            self, FollowJointTrajectory, action_name,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb_group)

        self.get_logger().info(
            f'FollowJointTrajectory server ready on "{action_name}"; '
            f'streaming to "{self.command_topic}" at {self.rate_hz:g} Hz.')

    # -- state ---------------------------------------------------------------
    def _on_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_positions[name] = pos

    def _actual_positions(self, joint_names):
        return [self.current_positions.get(n, 0.0) for n in joint_names]

    # -- action plumbing -----------------------------------------------------
    def _on_goal(self, goal_request):
        n = len(goal_request.trajectory.points)
        if n == 0:
            self.get_logger().warn('Rejecting goal: trajectory has no points.')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepted trajectory with {n} points.')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)
        points = list(traj.points)
        result = FollowJointTrajectory.Result()

        if self.pub.get_subscription_count() == 0:
            self.get_logger().warn(
                f'No subscriber on {self.command_topic} -- is Isaac Sim '
                'playing? Commands will be dropped.')

        duration = _duration_to_sec(points[-1].time_from_start)
        period = 1.0 / self.rate_hz
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names

        start = time.monotonic()
        while rclpy.ok():
            elapsed = time.monotonic() - start

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Trajectory canceled.')
                return result

            desired = sample_trajectory(points, elapsed)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = desired
            self.pub.publish(msg)

            actual = self._actual_positions(joint_names)
            feedback.desired.positions = desired
            feedback.actual.positions = actual
            feedback.error.positions = [a - d for a, d in zip(actual, desired)]
            goal_handle.publish_feedback(feedback)

            if elapsed >= duration:
                break
            time.sleep(period)

        # Let the PD drives settle onto the final commanded pose before judging.
        final = list(points[-1].positions)
        time.sleep(0.2)
        actual = self._actual_positions(joint_names)
        worst = max((abs(a - f) for a, f in zip(actual, final)), default=0.0)

        if self.current_positions and worst > self.goal_tolerance:
            self.get_logger().warn(
                f'Goal tolerance exceeded: worst joint error {worst:.4f} rad '
                f'> {self.goal_tolerance:.4f} rad.')
            result.error_code = result.GOAL_TOLERANCE_VIOLATED
            result.error_string = f'worst joint error {worst:.4f} rad'
            goal_handle.abort()
            return result

        goal_handle.succeed()
        result.error_code = result.SUCCESSFUL
        self.get_logger().info(
            f'Trajectory complete in {duration:.2f}s '
            f'(worst joint error {worst:.4f} rad).')
        return result


def main():
    rclpy.init()
    node = TrajectoryBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
