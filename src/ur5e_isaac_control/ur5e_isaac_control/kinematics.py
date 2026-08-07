#!/usr/bin/env python3
"""UR5e forward/inverse kinematics for the Isaac Sim pick project.

Thin wrapper around ikpy + the ur_description URDF. Exposes FK/IK on the 6 arm
joints in the same order the rest of the package uses (UR5E_JOINT_NAMES), with
the chain ending at the ROS-Industrial ``tool0`` frame. An optional TCP offset
lets you solve for a gripper tip that sits a fixed distance beyond tool0.

Frames (UR convention):
- ``base_link``: REP-103 aligned (X fwd, Y left, Z up). IK targets are in here.
- ``tool0``: all-zeros tool frame at the flange; its Z points *out* of the
  flange (the natural approach axis).
"""
import os
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from ikpy.chain import Chain

UR5E_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def _default_urdf_path():
    """Locate ur5e.urdf: installed share dir first, then the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('ur5e_isaac_control')
        p = os.path.join(share, 'urdf', 'ur5e.urdf')
        if os.path.exists(p):
            return p
    except Exception:
        pass
    # Fallback: source checkout (useful before an install/build).
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', 'urdf', 'ur5e.urdf'))


def _urdf_without_branch(urdf_path, drop_child_links=('ft_frame', 'base')):
    """Return a temp URDF path with the given tip/base sensor frames removed.

    ikpy builds a single chain and, at a branch, follows the first child joint
    it finds. The UR URDF branches at wrist_3_link (ft_frame vs flange->tool0)
    and at base_link (base vs base_link_inertia). Dropping the ft_frame/base
    sensor frames leaves a clean linear chain base_link -> ... -> tool0.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for joint in list(root.findall('joint')):
        child = joint.find('child')
        if child is not None and child.get('link') in drop_child_links:
            root.remove(joint)
    for link in list(root.findall('link')):
        if link.get('name') in drop_child_links:
            root.remove(link)
    fd, out = tempfile.mkstemp(suffix='.urdf', prefix='ur5e_ik_')
    os.close(fd)
    tree.write(out)
    return out


class UR5eKinematics:
    """FK/IK for the UR5e arm, operating on 6-vectors in UR5E_JOINT_NAMES order."""

    def __init__(self, urdf_path=None, tcp_z_offset=0.0):
        """tcp_z_offset: metres from tool0 to the grasp point along tool0 +Z
        (e.g. the Robotiq 2F-85 finger length). 0.0 == solve for tool0 itself.
        """
        urdf_path = urdf_path or _default_urdf_path()
        trimmed = _urdf_without_branch(urdf_path)
        try:
            base = Chain.from_urdf_file(trimmed, base_elements=['base_link'])
        finally:
            os.unlink(trimmed)

        # Activate only the 6 revolute arm joints; fixed links stay inactive so
        # the solver's DOFs match the real robot and ikpy stops warning.
        mask = [link.name in UR5E_JOINT_NAMES for link in base.links]
        self.chain = Chain(base.links, active_links_mask=mask)
        self._active_idx = [i for i, a in enumerate(mask) if a]
        assert len(self._active_idx) == 6, \
            f'expected 6 active joints, got {len(self._active_idx)}'

        # Fixed TCP offset applied in the tool0 frame (translation along +Z).
        self.tcp_offset = np.eye(4)
        self.tcp_offset[2, 3] = tcp_z_offset

    # -- internal padding between 6-vector and ikpy's full link vector --------
    def _pad(self, q6):
        v = np.zeros(len(self.chain.links))
        for i, val in zip(self._active_idx, q6):
            v[i] = val
        return v

    def _active(self, vfull):
        return np.array([vfull[i] for i in self._active_idx])

    # -- public API -----------------------------------------------------------
    def fk(self, q6):
        """6 joint angles (rad) -> 4x4 pose of the TCP in base_link."""
        T_tool0 = self.chain.forward_kinematics(self._pad(np.asarray(q6)))
        return T_tool0 @ self.tcp_offset

    def ik(self, target_pose, initial_q6=None, orientation=True):
        """4x4 TCP target in base_link -> 6 joint angles (rad).

        target_pose may be a 4x4 matrix. Set orientation=False to solve for
        position only (lets the solver pick a comfortable wrist).
        """
        target_pose = np.asarray(target_pose, dtype=float)
        # Convert a TCP target back to a tool0 target for the solver.
        T_tool0 = target_pose @ np.linalg.inv(self.tcp_offset)
        q0 = self._pad(initial_q6) if initial_q6 is not None else None
        sol = self.chain.inverse_kinematics(
            target_position=T_tool0[:3, 3],
            target_orientation=T_tool0[:3, :3] if orientation else None,
            orientation_mode='all' if orientation else None,
            initial_position=q0,
        )
        return self._active(sol)

    def pose_error(self, q6, target_pose):
        """(position error [m], orientation error [rad]) of fk(q6) vs target."""
        T = self.fk(q6)
        target_pose = np.asarray(target_pose, dtype=float)
        pos = np.linalg.norm(T[:3, 3] - target_pose[:3, 3])
        R = T[:3, :3] @ target_pose[:3, :3].T
        ang = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
        return pos, ang
