"""Isaac Sim Script Editor — where is the camera, relative to the robot's base?

Paste into Window > Script Editor and run. Prints the fixed transform from
``base_link`` to the camera's ROS *optical* frame, ready to paste into a
``static_transform_publisher``. Because the camera is bolted to the world, this
is a constant: measure once, reuse forever.

Two conventions have to be reconciled, and both are easy to get backwards:

1. **USD matrices are row-vector** (``p_world = p_local * M``), so the rotation
   block is the transpose of the column-vector convention ROS uses. Rather than
   transpose by hand and hope, this script asks USD to transform the camera's
   own axis directions (``TransformDir``) and builds the rotation matrix from
   the results. That is correct by construction whatever USD does internally.

2. **A USD camera looks down -Z with +Y up; a ROS optical frame looks down +Z
   with +Y down.** They are 180 degrees apart about X. Skip this and the tag
   lands behind the camera. Applied below as the optical_* axis flip.

The result still needs visual confirmation: ISSUES.md records that
USD link-frame *orientations* do not match the URDF's. Publish this transform,
render the detected pose as an RViz marker, and check it lands on the cube.
"""
import math

import omni.usd
from pxr import Usd, UsdGeom, Gf

# ============================== CONFIG ==============================
CAMERA_PATH = "/World/Camera"
BASE_PATH = "/World/ur5e/base_link"   # the robot's base; edit if yours differs
CAMERA_FRAME = "sim_camera"           # must match frame_id on /rgb
BASE_FRAME = "base_link"
# ====================================================================

stage = omni.usd.get_context().get_stage()


def world_xf(path):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError(f"no prim at {path} — check the Stage panel")
    return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default())


def quat_from_axes(x_axis, y_axis, z_axis):
    """Build a ROS (x, y, z, w) quaternion from three orthonormal axes given as
    the columns of the rotation matrix (i.e. where local X/Y/Z point)."""
    m = [[x_axis[0], y_axis[0], z_axis[0]],
         [x_axis[1], y_axis[1], z_axis[1]],
         [x_axis[2], y_axis[2], z_axis[2]]]
    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return x, y, z, w


m_cam = world_xf(CAMERA_PATH)
m_base = world_xf(BASE_PATH)

# USD row-vector composition: p_base = p_cam * M_cam * inverse(M_base)
m_cam_in_base = m_cam * m_base.GetInverse()

# Position: where the camera's origin sits in base_link coordinates.
t = m_cam_in_base.Transform(Gf.Vec3d(0, 0, 0))

# Orientation: transform the camera's own axes into base_link, then flip Y and
# Z to convert the USD camera frame into the ROS optical frame.
usd_x = m_cam_in_base.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
usd_y = m_cam_in_base.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
usd_z = m_cam_in_base.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()

opt_x = usd_x            # optical X = USD X   (right, in both)
opt_y = -usd_y           # optical Y = -USD Y  (down vs up)
opt_z = -usd_z           # optical Z = -USD Z  (into the scene vs out of it)

qx, qy, qz, qw = quat_from_axes(opt_x, opt_y, opt_z)

wc = m_cam.Transform(Gf.Vec3d(0, 0, 0))
wb = m_base.Transform(Gf.Vec3d(0, 0, 0))
print("")
print(f"[ext] camera in world  ({wc[0]:+.5f}, {wc[1]:+.5f}, {wc[2]:+.5f})")
print(f"[ext] base   in world  ({wb[0]:+.5f}, {wb[1]:+.5f}, {wb[2]:+.5f})")
print("")
print(f"[ext] USD camera axes, expressed in {BASE_FRAME}:")
print(f"        local +X -> ({usd_x[0]:+.4f}, {usd_x[1]:+.4f}, {usd_x[2]:+.4f})")
print(f"        local +Y -> ({usd_y[0]:+.4f}, {usd_y[1]:+.4f}, {usd_y[2]:+.4f})")
print(f"        local -Z -> ({-usd_z[0]:+.4f}, {-usd_z[1]:+.4f}, "
      f"{-usd_z[2]:+.4f})   <- the direction it is looking")
print("")
print(f"[ext] {BASE_FRAME} -> {CAMERA_FRAME} (ROS optical frame):")
print(f"        xyz  {t[0]:+.6f} {t[1]:+.6f} {t[2]:+.6f}")
print(f"        quat {qx:+.6f} {qy:+.6f} {qz:+.6f} {qw:+.6f}   (x y z w)")
print("")
print("[ext] paste this to publish it:")
print(f"ros2 run tf2_ros static_transform_publisher \\")
print(f"  --x {t[0]:.6f} --y {t[1]:.6f} --z {t[2]:.6f} \\")
print(f"  --qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} \\")
print(f"  --frame-id {BASE_FRAME} --child-frame-id {CAMERA_FRAME}")

# Sanity check: a top-down camera should be looking mostly downward, i.e. its
# viewing direction should have a strongly negative Z in base_link.
look = -usd_z
if look[2] > -0.5:
    print("")
    print(f"[ext] !! viewing direction Z is {look[2]:+.3f} — expected close to "
          f"-1 for a top-down camera. Check BASE_PATH is really the robot base "
          f"and that its USD orientation matches the URDF (see ISSUES.md).")
