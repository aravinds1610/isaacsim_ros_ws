#!/usr/bin/env python3
"""Print the gripper's geometry relative to the arm flange, off the live USD stage.

Run this INSIDE Isaac Sim: Window > Script Editor, paste, Run. It reads the
currently-open stage, so the scene must be loaded (it does NOT need to be
playing -- these are static transforms).

Why read the stage instead of a datasheet: the numbers that matter are whatever
*this* scene actually has, including however the gripper was parented and any
scaling applied on import. Those are the numbers MoveIt must agree with.

Output goes to the Script Editor pane *and* to OUT_PATH, because the Script
Editor does not reliably allow copying its output.

Output feeds M1b:
  * "tool0-relative" translation of the fingertips -> the ``grasp_tcp`` frame in
    ur5e.urdf, and kinematics.py's ``tcp_z_offset``
  * bounding-box sizes -> the <box>/<cylinder> collision primitives

Nothing on the stage is modified; this is read-only.
"""
import os

from pxr import Usd, UsdGeom, Gf
import omni.usd

# Where the report is written. Change if you want it somewhere else.
OUT_PATH = os.path.expanduser('~/isaacsim_ros_ws/gripper_measure.txt')

# Prims whose *name* contains one of these is reported. Cast wide -- it is far
# better to print an irrelevant prim than to silently miss the fingertip.
ARM_KEYS = ('tool0', 'flange', 'wrist_3', 'ft_frame')
GRIP_KEYS = ('robotiq', '2f', 'gripper', 'coupl', 'finger', 'knuckle', 'pad',
             'follower', 'driver')

# The frame everything is reported relative to, first match wins.
REF_PREFERENCE = ('tool0', 'flange', 'wrist_3')

_lines = []


def say(msg=''):
    """Print to the Script Editor and capture for the report file."""
    print(msg)
    _lines.append(str(msg))


def _fmt(v, unit_scale):
    """Metres, 4 dp, from stage units."""
    return '(' + ', '.join(f'{c * unit_scale:+.4f}' for c in v) + ')'


def report():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        say('No stage open.')
        return

    # Isaac Sim defaults to metres, but an imported asset can set otherwise.
    # Everything below is converted to metres so the URDF can use it directly.
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    say('=' * 78)
    say(f'stage metersPerUnit = {mpu}  '
        f'({"metres - values used as-is" if abs(mpu - 1.0) < 1e-9 else f"scaling all lengths by {mpu}"})')
    say('=' * 78)

    tc = Usd.TimeCode.Default()
    bbox_cache = UsdGeom.BBoxCache(
        tc, includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True)

    arm, grip = [], []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Xformable):
            continue
        low = prim.GetName().lower()
        if any(k in low for k in ARM_KEYS):
            arm.append(prim)
        elif any(k in low for k in GRIP_KEYS):
            grip.append(prim)

    if not arm:
        say('\n!! Found no prim named like tool0/flange/wrist_3.')
        say('   The arm may be named differently in this scene. Full prim list:')
        for prim in stage.Traverse():
            say(f'    {prim.GetPath()}')
        return

    say('\n--- ARM REFERENCE CANDIDATES ---')
    for p in arm:
        say(f'  {p.GetPath()}')

    # Pick the reference frame: prefer tool0, else flange, else wrist_3.
    ref = None
    for key in REF_PREFERENCE:
        for p in arm:
            if key in p.GetName().lower():
                ref = p
                break
        if ref is not None:
            break
    ref = ref or arm[0]

    ref_world = UsdGeom.Xformable(ref).ComputeLocalToWorldTransform(tc)
    ref_inv = ref_world.GetInverse()
    say(f'\nreference frame = {ref.GetPath()}')
    say(f'  world position {_fmt(ref_world.ExtractTranslation(), mpu)} m')

    if not grip:
        say('\n!! Found no gripper-like prim. Either the gripper is named')
        say('   unusually, or it is not on this stage. Prims under the arm:')
        for p in arm:
            for child in Usd.PrimRange(p):
                say(f'    {child.GetPath()}')
        return

    say('\n--- GRIPPER PRIMS, RELATIVE TO REFERENCE ---')
    say('(translation is what you want for grasp_tcp / tcp_z_offset;')
    say(' size is the world-aligned bounding box -> collision primitive dims)')
    say('')

    rows = []
    for p in sorted(grip, key=lambda x: str(x.GetPath())):
        world = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(tc)
        rel = world * ref_inv          # USD is row-vector: local * parentToWorld
        t = rel.ExtractTranslation()

        rng = bbox_cache.ComputeWorldBound(p).ComputeAlignedRange()
        size = None if rng.IsEmpty() else (rng.GetMax() - rng.GetMin())

        rows.append((str(p.GetPath()), t, size))
        say(f'{p.GetPath()}')
        say(f'    rel. translation {_fmt(t, mpu)} m'
            f'   |z| = {abs(t[2]) * mpu:.4f} m')
        if size is not None:
            say(f'    bbox size        {_fmt(size, mpu)} m')
        else:
            say('    bbox size        (no geometry - pure transform node)')

    # The fingertip furthest along the tool axis is the grasp point.
    tipish = [r for r in rows
              if any(k in r[0].lower() for k in ('finger', 'pad', 'tip'))]
    pool = tipish or rows
    far = max(pool, key=lambda r: abs(r[1][2]))
    say('')
    say('=' * 78)
    say('SUGGESTED grasp_tcp OFFSET (furthest finger-like prim along tool axis)')
    say(f'  prim  : {far[0]}')
    say(f'  xyz   : {_fmt(far[1], mpu)} m   <- grasp_tcp origin rel. {ref.GetName()}')
    say(f'  z only: {abs(far[1][2]) * mpu:.4f} m   <- kinematics.py tcp_z_offset')
    say('=' * 78)
    say('')
    say('Sanity-check that z against the real hardware before trusting it:')
    say('a Robotiq 2F-85 is on the order of 0.15 m from coupling face to tip.')
    say('If it looks wildly off, the gripper is probably parented somewhere')
    say('other than the reference frame above.')


def main():
    try:
        report()
    finally:
        # Always write whatever was gathered, even if traversal blew up
        # partway -- a partial report still tells us the prim naming.
        try:
            os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
            with open(OUT_PATH, 'w') as fh:
                fh.write('\n'.join(_lines) + '\n')
            print(f'\n>>> report written to {OUT_PATH}')
        except Exception as exc:                      # noqa: BLE001
            print(f'\n!! could not write {OUT_PATH}: {exc}')
            print('   (the output above is still complete - it just has to be '
                  'read from this pane)')


main()
