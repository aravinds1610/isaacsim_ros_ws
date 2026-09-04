#!/usr/bin/env python3
"""Measure static scene geometry for MoveIt planning-scene collision objects (M3).

Run INSIDE Isaac Sim: Window > Script Editor, paste, Run. Read-only.

WHY RECURSIVE, NOT TOP-LEVEL: a "group" in the Isaac UI is just a parent Xform
with the members reparented under it. Taking the group's bound gives the UNION of
its members -- for a bin grouped with its table that is one solid box spanning
both, which seals the bin's opening. MoveIt would then call every pose above the
bin a collision and M4's place motion could never plan, with "no plan found" as
the only symptom. So this walks down to leaf Gprims and measures each one; group
unions are printed too, but labelled reference-only.

WHY THE ROTATION CHECK USES THE ACCUMULATED MATRIX: prim transforms compose, so a
child that is axis-aligned in its own frame is not axis-aligned in world if the
group carries a yaw (or a scale, which multiplies into every child). A
world-axis-aligned bounding box is only the true extent for a prim that is
actually axis-aligned in world -- the trap isaac_gripper_measure.py fell into.
Every leaf therefore reports its accumulated world rotation and is flagged if it
is not within tolerance of axis-aligned; for those, the honest size comes from
the prim's own local range times its world scale, not from the world AABB.

WHY THE MESH STATS: a concave bin authored as a single mesh still has a sealed
AABB -- recursion does not help, because the mesh genuinely is one piece and a box
genuinely cannot represent a hollow. PhysX's convexDecomposition does not rescue
this: it is cooked to a binary cache, not to prims, so the hulls cannot be read
back here. MoveIt does not need them (FCL takes a concave triangle mesh directly),
so the choice is exact-mesh vs hand-authored walls, and triangle count plus the
AABB's proportions are what decide it. Both are printed per leaf.

Output: prints, and writes ~/isaacsim_ros_ws/scene_measure.txt.
"""
import math
import os

from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd

OUT_PATH = os.path.expanduser('~/isaacsim_ros_ws/scene_measure.txt')

# Subtrees to skip: the robot describes itself through the URDF, and the cube
# becomes an ATTACHED object at grasp time rather than static world geometry.
SKIP_PATHS = (
    '/World/ur5e',
    '/World/Cube',
)

# The frame every collision object must end up in. MoveIt's world is rooted at
# the URDF's base_link, so world-frame USD numbers have to be re-expressed here.
BASE_LINK_PATH = '/World/ur5e/base_link'

# PLAN.md's recorded base origin. Only a cross-check against the live stage --
# if these disagree the scene has moved and the camera extrinsic is stale too.
BASE_LINK_EXPECTED = (14.76516, 1.73420, 2.00951)

# Within this many radians of a multiple of 90 deg counts as axis-aligned.
ALIGN_TOL = 1e-4

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
    pitch = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(m[2][0]) < 0.999999:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    else:                                   # gimbal lock
        roll = math.atan2(-m[1][2], m[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def is_axis_aligned(rpy):
    """True when every angle sits within tolerance of a multiple of 90 deg."""
    for a in rpy:
        rem = math.fmod(abs(a), math.pi / 2.0)
        if min(rem, math.pi / 2.0 - rem) > ALIGN_TOL:
            return False
    return True


def base_link_origin(stage):
    """World translation of base_link, read live rather than trusted from notes."""
    prim = stage.GetPrimAtPath(BASE_LINK_PATH)
    if not prim or not prim.IsValid():
        say(f'!! {BASE_LINK_PATH} not found -- falling back to PLAN.md value.')
        say('   Every base_link figure below is only as good as that note.')
        return Gf.Vec3d(*BASE_LINK_EXPECTED), True
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    trans = Gf.Transform(m)
    t = trans.GetTranslation()
    rpy = quat_to_rpy(trans.GetRotation().GetQuat())
    if not is_axis_aligned(rpy):
        say('!! base_link is NOT axis-aligned in world '
            f'(rpy {rpy[0]:+.6f} {rpy[1]:+.6f} {rpy[2]:+.6f}).')
        say('   world->base_link is then not a pure translation and every')
        say('   base_link figure below is wrong. Stop and rederive.')
    drift = max(abs(t[i] - BASE_LINK_EXPECTED[i]) for i in range(3))
    if drift > 1e-3:
        say(f'!! base_link moved {drift:.4f} m from PLAN.md\'s recorded origin.')
        say('   Re-run isaac_camera_extrinsic.py -- the extrinsic is now stale.')
    return t, is_axis_aligned(rpy)


def mesh_stats(prim):
    """Point/face/triangle counts, or None for a non-mesh."""
    if not prim.IsA(UsdGeom.Mesh):
        return None
    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    if pts is None or counts is None:
        return {'points': 0, 'faces': 0, 'tris': 0, 'readable': False}
    # Fan-triangulation: an n-gon becomes n-2 triangles.
    tris = sum(max(0, c - 2) for c in counts)
    return {'points': len(pts), 'faces': len(counts), 'tris': tris,
            'readable': len(pts) > 0}


def rigid_body_ancestor(prim):
    """Nearest prim at or above `prim` carrying RigidBodyAPI, else None.

    The API is routinely authored on a parent Xform rather than on the mesh, so
    checking only the leaf would let a dynamic object be measured as furniture.
    """
    p = prim
    while p and p.IsValid() and str(p.GetPath()) != '/':
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return p
        p = p.GetParent()
    return None


def collision_note(prim):
    """What the physics side thinks this prim is. Isaac's collider does NOT feed
    MoveIt -- reported only so a moving rigid body is not mistaken for furniture.
    """
    bits = []
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        approx = prim.GetAttribute('physics:approximation')
        val = approx.Get() if approx and approx.HasAuthoredValue() else 'default'
        bits.append(f'collider={val}')
    rb = rigid_body_ancestor(prim)
    if rb is not None:
        where = 'here' if rb == prim else f'on ancestor {rb.GetPath()}'
        bits.append(f'!! RIGID BODY {where} -- moves, do NOT freeze into the scene')
    return ', '.join(bits)


def unique_name(prim):
    """A collision-object id unique across the stage.

    MoveIt keys world objects by id, so two prims both called `Cube` in
    different groups would overwrite each other in the planning scene -- one box
    silently vanishing. Derive from the path instead of the leaf name.
    """
    rel = str(prim.GetPath()).lstrip('/')
    if rel.startswith('World/'):
        rel = rel[len('World/'):]
    return rel.replace('/', '_')


def describe(prim, bbox_cache, base, indent, boxes):
    """Print one prim's geometry, recursing into children.

    Appends a dict per measured leaf to `boxes`. Leaves are appended whether or
    not they are axis-aligned -- the report downstream decides what is
    paste-ready and what needs a decision.
    """
    pad = '  ' * indent
    name = prim.GetName()
    path = str(prim.GetPath())

    if any(path == s or path.startswith(s + '/') for s in SKIP_PATHS):
        say(f'{pad}{name}   -- skipped ({path})')
        return

    kids = [c for c in prim.GetChildren() if UsdGeom.Imageable(c)]
    is_leaf_geom = prim.IsA(UsdGeom.Gprim)

    xf = UsdGeom.Xformable(prim)
    if not xf:
        say(f'{pad}{name}   [{prim.GetTypeName()}]  (not transformable)')
        return

    m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # Factorise rather than calling m.ExtractRotationQuat(): that method assumes
    # an orthonormal matrix and silently returns a skewed rotation once a scale
    # is present -- and scales are everywhere in this scene.
    trans = Gf.Transform(m)
    t = trans.GetTranslation()
    rpy = quat_to_rpy(trans.GetRotation().GetQuat())
    scale = trans.GetScale()
    aligned = is_axis_aligned(rpy)

    bbox = bbox_cache.ComputeWorldBound(prim)
    aligned_range = bbox.ComputeAlignedRange()          # world AABB
    local_range = bbox.GetRange()                       # pre-transform extent

    if aligned_range.IsEmpty():
        say(f'{pad}{name}   [{prim.GetTypeName()}]  (no geometry bound)')
        for c in kids:
            describe(c, bbox_cache, base, indent + 1, boxes)
        return

    tag = 'LEAF' if is_leaf_geom else 'GROUP'
    say(f'{pad}{name}   [{prim.GetTypeName()}]  {tag}')
    say(f'{pad}   path   {path}')
    say(f'{pad}   world  xyz="{t[0]:+.6f} {t[1]:+.6f} {t[2]:+.6f}"'
        f'  rpy="{rpy[0]:+.6f} {rpy[1]:+.6f} {rpy[2]:+.6f}"')
    if abs(scale[0] - 1) > 1e-6 or abs(scale[1] - 1) > 1e-6 or abs(scale[2] - 1) > 1e-6:
        say(f'{pad}   scale  {scale[0]:+.6f} {scale[1]:+.6f} {scale[2]:+.6f}'
            f'   (accumulated -- already folded into the sizes below)')

    note = collision_note(prim)
    if note:
        say(f'{pad}   physx  {note}')

    ms = mesh_stats(prim)
    if ms is not None:
        say(f'{pad}   mesh   {ms["points"]} points, {ms["faces"]} faces, '
            f'{ms["tris"]} triangles'
            + ('' if ms['readable'] else '   !! points not readable'))

    lo, hi = aligned_range.GetMin(), aligned_range.GetMax()
    size_w = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    ctr_w = ((hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2, (hi[2] + lo[2]) / 2)
    ctr_b = (ctr_w[0] - base[0], ctr_w[1] - base[1], ctr_w[2] - base[2])

    if not is_leaf_geom:
        say(f'{pad}   union  size {size_w[0]:.6f} {size_w[1]:.6f} {size_w[2]:.6f}'
            '   << REFERENCE ONLY, not a collision box')
        say(f'{pad}          (union of {len(kids)} member(s) -- use the leaves)')
        for c in kids:
            describe(c, bbox_cache, base, indent + 1, boxes)
        return

    say(f'{pad}   AABB   min {lo[0]:+.6f} {lo[1]:+.6f} {lo[2]:+.6f}')
    say(f'{pad}          max {hi[0]:+.6f} {hi[1]:+.6f} {hi[2]:+.6f}')

    size_o = None
    if aligned:
        say(f'{pad}   size   {size_w[0]:.6f} {size_w[1]:.6f} {size_w[2]:.6f}')
        say(f'{pad}   centre world     {ctr_w[0]:+.6f} {ctr_w[1]:+.6f} {ctr_w[2]:+.6f}')
        say(f'{pad}   centre base_link {ctr_b[0]:+.6f} {ctr_b[1]:+.6f} {ctr_b[2]:+.6f}')
    else:
        # World AABB overstates a rotated prim. Its true extent is the local
        # range scaled by the accumulated world scale.
        llo, lhi = local_range.GetMin(), local_range.GetMax()
        size_o = ((lhi[0] - llo[0]) * abs(scale[0]),
                  (lhi[1] - llo[1]) * abs(scale[1]),
                  (lhi[2] - llo[2]) * abs(scale[2]))
        say(f'{pad}   !! NOT axis-aligned in world -- the AABB above overstates it.')
        say(f'{pad}   oriented size    {size_o[0]:.6f} {size_o[1]:.6f} {size_o[2]:.6f}')
        say(f'{pad}   centre base_link {ctr_b[0]:+.6f} {ctr_b[1]:+.6f} {ctr_b[2]:+.6f}')
        say(f'{pad}   -> author this box WITH the rpy above, not as an AABB.')

    boxes.append({'name': unique_name(prim),
                  'path': path,
                  'size': size_w if aligned else size_o,
                  'centre': ctr_b, 'rpy': rpy, 'aligned': aligned,
                  'dynamic': rigid_body_ancestor(prim) is not None,
                  'tris': ms['tris'] if ms else 0})

    for c in kids:
        describe(c, bbox_cache, base, indent + 1, boxes)


def report():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        say('No stage open.')
        return

    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    say('=' * 78)
    say(f'stage metersPerUnit = {mpu}')
    if abs(mpu - 1.0) > 1e-9:
        say('!! Not 1.0 -- every length below is in stage units, NOT metres.')
    say('=' * 78)

    base, base_ok = base_link_origin(stage)
    say(f'\nbase_link world origin  {base[0]:+.6f} {base[1]:+.6f} {base[2]:+.6f}')
    say('world -> base_link is a pure translation'
        if base_ok else '!! base_link is rotated -- see warning above')
    say('')

    world = stage.GetPrimAtPath('/World')
    if not world or not world.IsValid():
        say('!! /World not found.')
        return

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=False)

    say('=' * 78)
    say('SCENE TREE  (indent = USD parenting; groups show as GROUP)')
    say('=' * 78)

    boxes = []
    for child in world.GetChildren():
        if not UsdGeom.Imageable(child):
            continue
        say('')
        describe(child, bbox_cache, base, 0, boxes)

    say('')
    say('=' * 78)
    say('PASTE-READY BOXES  (base_link frame, metres)')
    say('=' * 78)
    say('# Static, axis-aligned, primitive-shaped leaves only. Everything else')
    say('# is listed underneath -- those need a decision, not a copy-paste.')
    say('COLLISION_BOXES = [')
    clean = [b for b in boxes
             if b['aligned'] and b['size'] and not b['dynamic'] and b['tris'] <= 12]
    for b in clean:
        say(f'    dict(name={b["name"]!r},')
        say(f'         size=({b["size"][0]:.6f}, {b["size"][1]:.6f}, '
            f'{b["size"][2]:.6f}),')
        say(f'         centre=({b["centre"][0]:+.6f}, {b["centre"][1]:+.6f}, '
            f'{b["centre"][2]:+.6f})),')
    say(']')
    if not clean:
        say('# (none -- every leaf needs a decision, see below)')

    needs_thought = [b for b in boxes if b not in clean]
    if needs_thought:
        say('')
        say('NEEDS A DECISION:')
        for b in needs_thought:
            why = []
            if b['dynamic']:
                why.append('DYNAMIC rigid body -> not furniture; skip it, or handle '
                           'as an attached object')
            if not b['aligned']:
                why.append(f'rotated (rpy {b["rpy"][0]:+.4f} {b["rpy"][1]:+.4f} '
                           f'{b["rpy"][2]:+.4f}) -> box + orientation, size above '
                           'is the oriented one')
            if b['tris'] > 12:
                why.append(f'{b["tris"]} tris -> exact mesh, or hand-authored walls '
                           'if it is a hollow container')
            say(f'    {b["path"]}')
            for w in why:
                say(f'        - {w}')


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
