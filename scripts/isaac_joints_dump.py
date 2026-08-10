#!/usr/bin/env python3
"""Dump every physics joint on the stage, with its two local frames.

Run INSIDE Isaac Sim: Window > Script Editor, paste, Run. Read-only.

WHY THIS AND NOT A TRANSFORM COMPARISON: in a USD articulation the links are
flat siblings under the robot prim -- /World/ur5e/base_link, .../wrist_3_link,
.../Gripper are all peers. The kinematic chain lives in PhysicsJoint prims under
the `joints` Scope, NOT in USD parenting. So comparing two links'
ComputeLocalToWorldTransform tells you nothing about how they are connected, and
rigid-body xforms hold authored values rather than simulated poses anyway.

The joint prim carries exactly what a URDF <joint><origin> needs:
    physics:body0      parent link
    physics:body1      child link
    physics:localPos0  child origin expressed in body0's frame   <-- the mount
    physics:localRot0  child orientation in body0's frame

Output feeds M1b: find the joint whose body0 is wrist_3_link (or a tool frame)
and whose body1 is the gripper base -- localPos0/localRot0 is the tool0->gripper
mount transform for the parent xacro.
"""
import math
import os

from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd

OUT_PATH = os.path.expanduser('~/isaacsim_ros_ws/joints_dump.txt')

_lines = []


def say(msg=''):
    print(msg)
    _lines.append(str(msg))


def quat_to_rpy(q):
    """URDF fixed-axis roll-pitch-yaw (R = Rz*Ry*Rx) from a USD quaternion.

    USD's Gf matrices are row-vector (v' = v*M), URDF/ROS maths is column-vector
    (v' = M*v), so the 3x3 is transposed before the standard extraction.
    """
    m = Gf.Matrix3d(Gf.Rotation(q)).GetTranspose()
    r02 = m[0][2]
    # Clamp guards against a tiny out-of-range value from float error.
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(m[2][0]) < 0.999999:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:                                   # gimbal lock
        roll = math.atan2(-m[1][2], m[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def targets(prim, name):
    rel = prim.GetRelationship(name)
    return [str(p) for p in rel.GetTargets()] if rel else []


def get(prim, name, default=None):
    at = prim.GetAttribute(name)
    return at.Get() if at and at.HasAuthoredValue() else default


def report():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        say('No stage open.')
        return

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    say('=' * 78)
    say(f'stage metersPerUnit = {mpu}')
    say('=' * 78)

    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    if not joints:
        say('\n!! No UsdPhysics.Joint prims found. Prim types present:')
        seen = {}
        for p in stage.Traverse():
            seen[p.GetTypeName()] = seen.get(p.GetTypeName(), 0) + 1
        for k, v in sorted(seen.items(), key=lambda kv: -kv[1]):
            say(f'   {v:5d}  {k}')
        return

    say(f'\nfound {len(joints)} physics joint(s)\n')

    for j in sorted(joints, key=lambda p: str(p.GetPath())):
        b0 = targets(j, 'physics:body0')
        b1 = targets(j, 'physics:body1')
        p0 = get(j, 'physics:localPos0', Gf.Vec3f(0, 0, 0))
        p1 = get(j, 'physics:localPos1', Gf.Vec3f(0, 0, 0))
        r0 = get(j, 'physics:localRot0', Gf.Quatf(1, 0, 0, 0))
        r1 = get(j, 'physics:localRot1', Gf.Quatf(1, 0, 0, 0))

        say(f'{j.GetPath()}   [{j.GetTypeName()}]')
        say(f'    body0 (parent): {b0 if b0 else "(none)"}')
        say(f'    body1 (child) : {b1 if b1 else "(none)"}')

        for tag, pos, rot in (('0', p0, r0), ('1', p1, r1)):
            xyz = tuple(c * mpu for c in pos)
            rpy = quat_to_rpy(rot)
            say(f'    localPos{tag} xyz="{xyz[0]:+.6f} {xyz[1]:+.6f} {xyz[2]:+.6f}"')
            say(f'    localRot{tag} rpy="{rpy[0]:+.6f} {rpy[1]:+.6f} {rpy[2]:+.6f}"'
                f'   (quat wxyz {rot.GetReal():+.4f} '
                f'{rot.GetImaginary()[0]:+.4f} {rot.GetImaginary()[1]:+.4f} '
                f'{rot.GetImaginary()[2]:+.4f})')

        axis = get(j, 'physics:axis')
        if axis:
            say(f'    axis: {axis}')
        say('')

    # Highlight the joint(s) that bridge the arm and the gripper -- that is the
    # single number the parent xacro needs.
    say('=' * 78)
    say('LIKELY ARM -> GRIPPER MOUNT JOINT(S)')
    say('=' * 78)
    hits = []
    for j in joints:
        blob = (' '.join(targets(j, 'physics:body0'))
                + ' ' + ' '.join(targets(j, 'physics:body1'))).lower()
        if ('gripper' in blob or 'robotiq' in blob or '2f' in blob) and \
           ('wrist' in blob or 'tool' in blob or 'flange' in blob):
            hits.append(j)
    if hits:
        for j in hits:
            say(f'  {j.GetPath()}')
            say(f'    parent {targets(j, "physics:body0")}')
            say(f'    child  {targets(j, "physics:body1")}')
    else:
        say('  none matched automatically -- read the full list above and find')
        say('  the joint whose body0 is a wrist/tool link and body1 is the')
        say('  gripper base.')


def main():
    try:
        report()
    finally:
        try:
            os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
            with open(OUT_PATH, 'w') as fh:
                fh.write('\n'.join(_lines) + '\n')
            print(f'\n>>> written to {OUT_PATH}')
        except Exception as exc:                      # noqa: BLE001
            print(f'\n!! could not write {OUT_PATH}: {exc}')


main()
