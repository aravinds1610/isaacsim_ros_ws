"""Isaac Sim Script Editor — re-bake a FixedJoint to the current placement.

**Run with the simulation STOPPED.**

Why this is needed: a ``FixedJoint`` stores the *relative pose between two
bodies*, captured when the joint was authored. Dragging a prim in the viewport
changes its stage transform but leaves that stored pose untouched. So on Stop
you see where you dragged it, and on Play PhysX restores the joint's old
relative pose -- the body appears to jump or hover.

This rewrites the joint's local frames so the constraint matches where the
bodies sit right now:

    localPos1/localRot1 := identity          (the child's own origin)
    localPos0/localRot0 := inverse(T_world_body0) * T_world_body1

DRY_RUN prints what it would write without changing anything. Set it False to
apply.
"""
import math

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

# ============================== CONFIG ==============================
JOINT_PATH = ""      # "" = every FixedJoint on the stage; or one exact path
DRY_RUN = False       # set False to actually write
# ====================================================================

stage = omni.usd.get_context().get_stage()


def world_xf(path):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        return None
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())


def mat_to_pos_quat(m):
    """4x4 (USD row-vector) -> (Gf.Vec3f position, Gf.Quatf rotation)."""
    xf = Gf.Transform(m)
    t = xf.GetTranslation()
    q = xf.GetRotation().GetQuat()
    return (Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])),
            Gf.Quatf(float(q.GetReal()),
                     Gf.Vec3f(*[float(v) for v in q.GetImaginary()])))


targets = []
if JOINT_PATH:
    p = stage.GetPrimAtPath(JOINT_PATH)
    if not p:
        raise RuntimeError(f"no prim at {JOINT_PATH}")
    targets = [p]
else:
    targets = [p for p in stage.Traverse() if p.IsA(UsdPhysics.FixedJoint)]

if not targets:
    raise RuntimeError("no FixedJoint found on the stage")

print(f"\n{'DRY RUN — nothing written' if DRY_RUN else 'APPLYING CHANGES'}")

for prim in targets:
    j = UsdPhysics.FixedJoint(prim)
    b0 = j.GetBody0Rel().GetTargets()
    b1 = j.GetBody1Rel().GetTargets()
    print(f"\n=== {prim.GetPath()}")
    print(f"    body0 {[str(x) for x in b0]}")
    print(f"    body1 {[str(x) for x in b1]}")
    print(f"    before: localPos0 {j.GetLocalPos0Attr().Get()} "
          f"localRot0 {j.GetLocalRot0Attr().Get()}")
    print(f"            localPos1 {j.GetLocalPos1Attr().Get()} "
          f"localRot1 {j.GetLocalRot1Attr().Get()}")

    if not b1:
        print("    !! body1 is empty — cannot re-bake, skipping")
        continue

    m1 = world_xf(str(b1[0]))
    if m1 is None:
        print(f"    !! body1 prim {b1[0]} not found, skipping")
        continue

    # body0 empty means the joint anchors to the world frame.
    if b0:
        m0 = world_xf(str(b0[0]))
        if m0 is None:
            print(f"    !! body0 prim {b0[0]} not found, skipping")
            continue
    else:
        m0 = Gf.Matrix4d(1.0)
        print("    (body0 empty — anchoring to world)")

    # USD row-vector convention: relative = M1 * inverse(M0)
    rel = m1 * m0.GetInverse()
    pos0, rot0 = mat_to_pos_quat(rel)

    print(f"    after : localPos0 {pos0} localRot0 {rot0}")
    print(f"            localPos1 (0, 0, 0) localRot1 (1, 0, 0, 0)")

    if not DRY_RUN:
        j.CreateLocalPos0Attr().Set(pos0)
        j.CreateLocalRot0Attr().Set(rot0)
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        print("    -> written")

print("\nNow press Play. The body should stay exactly where it is on Stop.")
