#!/usr/bin/env python3
"""Interactive joint controller for the UR5e in Isaac Sim.

A learning tool: one slider per joint. Drag a slider and the arm follows it
live in the Isaac Sim viewport. The panel publishes sensor_msgs/JointState to
/joint_command continuously (so the Isaac Sim Articulation Controller keeps
tracking the sliders) and reads /joint_states to show — and to snap to — the
arm's real position, so nothing jumps on startup.

Run (with Isaac Sim playing):
    ros2 run ur5e_isaac_control joint_controller_gui
"""
import math
import tkinter as tk
from tkinter import ttk

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

# UR5e revolute joints travel +/- 2*pi. Give sliders that full range so you can
# explore the whole workspace while learning.
JOINT_MIN = -2.0 * math.pi
JOINT_MAX = 2.0 * math.pi

HOME_POSE = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]


class JointControllerNode(Node):
    """ROS2 side: publishes /joint_command, tracks latest /joint_states."""

    def __init__(self, command_topic, state_topic):
        super().__init__('ur5e_joint_controller_gui')
        self.joint_names = UR5E_JOINT_NAMES
        self.latest_state = None  # list of positions in our joint order

        self.pub = self.create_publisher(JointState, command_topic, 10)
        self.create_subscription(JointState, state_topic, self._on_state, 10)

    def _on_state(self, msg: JointState):
        try:
            self.latest_state = [
                msg.position[msg.name.index(j)] for j in self.joint_names
            ]
        except (ValueError, IndexError):
            pass  # message doesn't carry all our joints yet

    def publish(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(p) for p in positions]
        self.pub.publish(msg)

    def subscriber_count(self):
        return self.pub.get_subscription_count()


class JointControllerGUI:
    """Tk side: sliders + live readouts, driving the ROS2 node."""

    PUBLISH_HZ = 30.0

    def __init__(self, node: JointControllerNode):
        self.node = node
        self.publishing = True

        self.root = tk.Tk()
        self.root.title('UR5e Joint Controller (Isaac Sim)')
        self.root.minsize(560, 380)

        self.slider_vars = []
        self.value_labels = []
        self._build_ui()

        # Snap sliders to the arm's real pose once the first state arrives, so
        # enabling publish doesn't yank the arm to a stale slider value.
        self._synced_once = False
        self.root.after(200, self._sync_when_ready)
        self.root.after(int(1000 / self.PUBLISH_HZ), self._tick)

    def _build_ui(self):
        header = ttk.Frame(self.root, padding=(12, 10, 12, 4))
        header.pack(fill='x')
        self.status_var = tk.StringVar(value='Waiting for /joint_states ...')
        ttk.Label(header, textvariable=self.status_var).pack(side='left')

        body = ttk.Frame(self.root, padding=(12, 4))
        body.pack(fill='both', expand=True)
        body.columnconfigure(1, weight=1)

        for row, name in enumerate(UR5E_JOINT_NAMES):
            ttk.Label(body, text=name, width=18, anchor='w').grid(
                row=row, column=0, sticky='w', pady=4)

            var = tk.DoubleVar(value=0.0)
            scale = ttk.Scale(
                body, from_=JOINT_MIN, to=JOINT_MAX, orient='horizontal',
                variable=var,
                command=lambda _v, r=row: self._update_value_label(r))
            scale.grid(row=row, column=1, sticky='ew', padx=8, pady=4)
            self.slider_vars.append(var)

            lbl = ttk.Label(body, text='', width=18, anchor='e',
                            font=('TkFixedFont',))
            lbl.grid(row=row, column=2, sticky='e')
            self.value_labels.append(lbl)
            self._update_value_label(row)

        controls = ttk.Frame(self.root, padding=(12, 8, 12, 12))
        controls.pack(fill='x')

        self.publish_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(controls, text='Publish to /joint_command',
                        variable=self.publish_var,
                        command=self._on_publish_toggle).pack(side='left')

        ttk.Button(controls, text='Sync to current',
                   command=self._sync_to_current).pack(side='right', padx=4)
        ttk.Button(controls, text='Home',
                   command=self._go_home).pack(side='right', padx=4)

    def _update_value_label(self, row):
        rad = self.slider_vars[row].get()
        deg = math.degrees(rad)
        self.value_labels[row].configure(text=f'{rad:+.3f} rad / {deg:+6.1f} deg')

    def _set_sliders(self, positions):
        for var, pos in zip(self.slider_vars, positions):
            var.set(pos)
        for row in range(len(self.slider_vars)):
            self._update_value_label(row)

    def _sync_when_ready(self):
        if not self._synced_once and self.node.latest_state is not None:
            self._set_sliders(self.node.latest_state)
            self._synced_once = True
        if not self._synced_once:
            self.root.after(200, self._sync_when_ready)

    def _sync_to_current(self):
        if self.node.latest_state is not None:
            self._set_sliders(self.node.latest_state)

    def _go_home(self):
        self._set_sliders(HOME_POSE)

    def _on_publish_toggle(self):
        self.publishing = self.publish_var.get()

    def _tick(self):
        """Spin ROS once and (optionally) publish the current slider pose."""
        rclpy.spin_once(self.node, timeout_sec=0.0)

        subs = self.node.subscriber_count()
        state = 'connected' if subs > 0 else 'NO subscriber on /joint_command'
        pub = 'ON' if self.publishing else 'OFF'
        self.status_var.set(f'publish: {pub}   |   Isaac Sim: {state} ({subs})')

        if self.publishing and self._synced_once:
            self.node.publish([v.get() for v in self.slider_vars])

        self.root.after(int(1000 / self.PUBLISH_HZ), self._tick)

    def run(self):
        self.root.protocol('WM_DELETE_WINDOW', self.root.quit)
        self.root.mainloop()


def main():
    rclpy.init()
    node = JointControllerNode('/joint_command', '/joint_states')
    node.get_logger().info(
        'Joint controller GUI up. Make sure Isaac Sim is playing.')
    gui = JointControllerGUI(node)
    try:
        gui.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
