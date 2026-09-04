#!/usr/bin/env python3
import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

UR5E_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# A couple of preset poses (radians) for the --demo sweep.
HOME_POSE = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
BENT_POSE = [0.5, -1.2, 1.2, -1.5, -1.57, 0.0]

# Waypoint sequence (radians) mimicking a pick-and-place motion: approach the
# pick side, reach down, lift, swing to the place side, reach down, lift, home.
PICK_APPROACH = [0.8, -1.0, 1.0, -1.5, -1.57, 0.0]
PICK_DOWN = [0.8, -0.6, 1.3, -2.2, -1.57, 0.0]
PLACE_APPROACH = [-0.8, -1.0, 1.0, -1.5, -1.57, 0.0]
PLACE_DOWN = [-0.8, -0.6, 1.3, -2.2, -1.57, 0.0]
PICK_PLACE_SEQUENCE = [
    (PICK_APPROACH, 2.0, 0.0),
    (PICK_DOWN, 1.5, 1.0),      # reach down, pause to mimic grasping
    (PICK_APPROACH, 1.5, 0.0),  # lift back up
    (PLACE_APPROACH, 2.0, 0.0),  # swing over to the place side
    (PLACE_DOWN, 1.5, 1.0),     # reach down, pause to mimic releasing
    (PLACE_APPROACH, 1.5, 0.0),  # lift back up
    (HOME_POSE, 2.0, 0.0),
]


class JointCommandPublisher(Node):

    def __init__(self, joint_names, command_topic, state_topic, rate_hz):
        super().__init__('ur5e_joint_command_publisher')
        self.joint_names = joint_names
        self.rate_hz = rate_hz
        self.current_positions = None

        self.pub = self.create_publisher(JointState, command_topic, 10)
        self.state_sub = self.create_subscription(
            JointState, state_topic, self._on_state, 10)

    def _on_state(self, msg: JointState):
        try:
            self.current_positions = [
                msg.position[msg.name.index(j)] for j in self.joint_names
            ]
        except ValueError:
            pass  # message doesn't contain all our joints yet

    def wait_for_state(self, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while self.current_positions is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_positions

    def wait_for_command_subscriber(self, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pub.get_subscription_count() > 0

    def move_to(self, target_positions, duration_sec):
        start = self.current_positions or target_positions
        steps = max(int(duration_sec * self.rate_hz), 1)
        period = 1.0 / self.rate_hz

        for step in range(1, steps + 1):
            alpha = step / steps
            interp = [
                s + (t - s) * alpha for s, t in zip(start, target_positions)
            ]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = interp
            self.pub.publish(msg)
            self.current_positions = interp
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


def parse_positions(text):
    values = [float(v) for v in text.split(',')]
    if len(values) != len(UR5E_JOINT_NAMES):
        raise argparse.ArgumentTypeError(
            f'Expected {len(UR5E_JOINT_NAMES)} comma-separated joint '
            f'positions (radians), got {len(values)}')
    return values


def main():
    parser = argparse.ArgumentParser(
        description='Move a UR5e in Isaac Sim to a target joint pose over ROS2.')
    parser.add_argument(
        '--positions', type=parse_positions, default=None,
        help='Comma-separated target joint positions in radians, e.g. '
             '"0,-1.57,1.57,-1.57,-1.57,0"')
    parser.add_argument(
        '--duration', type=float, default=3.0,
        help='Seconds to take moving to the target pose (default: 3.0)')
    parser.add_argument(
        '--demo', action='store_true',
        help='Ignore --positions and sweep between two preset poses once')
    parser.add_argument(
        '--pick-place', action='store_true',
        help='Ignore --positions and run a preset pick-and-place waypoint sequence')
    parser.add_argument(
        '--command-topic', default='/joint_command')
    parser.add_argument(
        '--state-topic', default='/joint_states')
    parser.add_argument(
        '--rate', type=float, default=60.0, help='Publish rate in Hz')
    args = parser.parse_args()

    if not args.demo and not args.pick_place and args.positions is None:
        parser.error(
            'pass --positions "j1,j2,j3,j4,j5,j6", or use --demo / --pick-place')

    rclpy.init()
    node = JointCommandPublisher(
        UR5E_JOINT_NAMES, args.command_topic, args.state_topic, args.rate)

    node.get_logger().info(
        f'Waiting for current joint state on {args.state_topic} ...')
    if node.wait_for_state() is None:
        node.get_logger().warn(
            'No joint state received; starting interpolation from the '
            'target pose itself (first published point will be the target).')

    node.get_logger().info(
        f'Waiting for a subscriber on {args.command_topic} ...')
    if not node.wait_for_command_subscriber():
        node.get_logger().warn(
            'No subscriber matched yet; commands may be dropped.')

    if args.demo:
        for pose in (BENT_POSE, HOME_POSE):
            node.get_logger().info(f'Moving to {pose}')
            node.move_to(pose, args.duration)
    elif args.pick_place:
        for pose, duration, pause_sec in PICK_PLACE_SEQUENCE:
            node.get_logger().info(f'Moving to {pose}')
            node.move_to(pose, duration)
            if pause_sec:
                time.sleep(pause_sec)
    else:
        node.get_logger().info(f'Moving to {args.positions}')
        node.move_to(args.positions, args.duration)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
