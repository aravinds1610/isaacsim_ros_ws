"""Isaac Sim Script Editor — will the AprilTag actually be big enough to detect?

Paste into Window > Script Editor, set the CONFIG block, run. Reads the camera
prim's lens settings and its distance to the cube, and works out how many pixels
across the tag will be in the published image.

Why this matters: a 36h11 tag is 8 cells wide (6 data + a 1-cell black border
each side). Decoding samples each cell, so you need several pixels per cell. Too
few and the tag is silently rejected -- the failure looks exactly like a broken
pipeline, so it is much cheaper to check the arithmetic now.
"""
import omni.usd
from pxr import Usd, UsdGeom, Gf

# ============================== CONFIG ==============================
CAMERA_PATH = "/World/Camera"   # copy the exact path from the Stage panel
CUBE_PATH   = "/World/Cube"
TAG_SIZE_M  = 0.03692           # from isaac_tag_decal.py
RENDER_W    = 1280              # resolution you will publish at
RENDER_H    = 720
# ====================================================================

stage = omni.usd.get_context().get_stage()

cam_prim = stage.GetPrimAtPath(CAMERA_PATH)
if not cam_prim:
    raise RuntimeError(f"no prim at {CAMERA_PATH} — check the Stage panel")
cam = UsdGeom.Camera(cam_prim)

# An orthographic camera has no perspective, so apparent size carries no depth
# information and solvePnP cannot recover a pose from it -- it returns numbers
# that look plausible and mean nothing. Isaac still publishes a pinhole
# camera_info for it, so nothing downstream complains. Caught this the hard way
# on 2026-08-11: focalLength changes did nothing and the published fx was 3x
# the value the image actually behaved like.
projection = cam.GetProjectionAttr().Get()
if projection != 'perspective':
    raise RuntimeError(
        f"{CAMERA_PATH} projection is '{projection}', must be 'perspective'. "
        f"Fix with: UsdGeom.Camera(prim).GetProjectionAttr()"
        f".Set(UsdGeom.Tokens.perspective)")

focal = cam.GetFocalLengthAttr().Get()
h_ap = cam.GetHorizontalApertureAttr().Get()
v_ap = cam.GetVerticalApertureAttr().Get()

# focalLength and aperture share the same units in USD, so their RATIO is
# unitless -- which makes this correct whatever the stage units are.
fx = RENDER_W * focal / h_ap
fy = RENDER_H * focal / v_ap

cam_xf = UsdGeom.Xformable(cam_prim).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default())
cam_pos = Gf.Vec3d(cam_xf.ExtractTranslation())

cube_prim = stage.GetPrimAtPath(CUBE_PATH)
if not cube_prim:
    raise RuntimeError(f"no prim at {CUBE_PATH}")
cube_xf = UsdGeom.Xformable(cube_prim).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default())
cube_pos = Gf.Vec3d(cube_xf.ExtractTranslation())

dist = (cam_pos - cube_pos).GetLength()
tag_px = fx * TAG_SIZE_M / dist
px_per_cell = tag_px / 8.0

print("")
print(f"[cam] camera at   ({cam_pos[0]:+.4f}, {cam_pos[1]:+.4f}, "
      f"{cam_pos[2]:+.4f}) m")
print(f"[cam] cube at     ({cube_pos[0]:+.4f}, {cube_pos[1]:+.4f}, "
      f"{cube_pos[2]:+.4f}) m")
print(f"[cam] distance    {dist:.4f} m")
print(f"[cam] focalLength {focal}   horizAperture {h_ap}   "
      f"vertAperture {v_ap}")
print(f"[cam] resolution  {RENDER_W} x {RENDER_H}")
print(f"[cam] fx {fx:.1f} px, fy {fy:.1f} px")

# fx and fy should match, or the image is stretched and pose estimates skew.
if abs(fx - fy) / max(fx, fy) > 0.02:
    want_v = h_ap * RENDER_H / RENDER_W
    print(f"[cam] !! non-square pixels (fx != fy). Set verticalAperture to "
          f"{want_v:.4f} to match the {RENDER_W}x{RENDER_H} aspect ratio.")

print("")
print(f"[cam] >>> tag will be {tag_px:.1f} px across "
      f"({px_per_cell:.1f} px per cell)")
if tag_px >= 40:
    print("[cam] >>> GOOD — comfortable margin for reliable detection.")
elif tag_px >= 30:
    print("[cam] >>> OK — should work, but little margin. Tighter is better.")
else:
    print("[cam] >>> TOO SMALL — detection will be unreliable or fail entirely.")
    need = 40.0 / tag_px
    print(f"[cam] >>> need ~{need:.1f}x more: crop tighter, raise focalLength, "
          f"lower the camera, or increase resolution.")
