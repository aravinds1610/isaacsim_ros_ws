"""Isaac Sim Script Editor — set the camera's lens in one shot, then verify.

The Property panel appears to adjust aperture fields as you edit them, so
setting focalLength / horizontalAperture / verticalAperture by hand tends to
land somewhere other than where you aimed. This writes all three atomically and
immediately reads back what actually stuck.

verticalAperture is derived, never typed: it must be
horizontalAperture * RENDER_H / RENDER_W, or the image is stretched and any
measurement taken from it is skewed.
"""
import omni.usd
from pxr import Usd, UsdGeom, Gf

# ============================== CONFIG ==============================
CAMERA_PATH = "/World/Camera"
CUBE_PATH   = "/World/Cube"
TAG_SIZE_M  = 0.03692
RENDER_W    = 1280
RENDER_H    = 720

FOCAL_LENGTH = 80.0     # bigger = more zoomed in = bigger tag, narrower view
HORIZ_APERTURE = 27.8254
# ====================================================================

VERT_APERTURE = HORIZ_APERTURE * RENDER_H / RENDER_W

stage = omni.usd.get_context().get_stage()
cam_prim = stage.GetPrimAtPath(CAMERA_PATH)
if not cam_prim:
    raise RuntimeError(f"no prim at {CAMERA_PATH}")
cam = UsdGeom.Camera(cam_prim)

# Perspective is not optional: an orthographic camera has no perspective, so
# apparent size carries no depth information and solvePnP cannot recover a pose
# from it. Isaac publishes a pinhole camera_info regardless, so the failure is
# silent. The camera was authored orthographic and it cost a long hunt on
# 2026-08-11 -- forced here so it cannot regress.
cam.GetProjectionAttr().Set(UsdGeom.Tokens.perspective)
cam.GetFocalLengthAttr().Set(FOCAL_LENGTH)
cam.GetHorizontalApertureAttr().Set(HORIZ_APERTURE)
cam.GetVerticalApertureAttr().Set(VERT_APERTURE)

# ---- read back what actually stuck, rather than trusting the writes ----
focal = cam.GetFocalLengthAttr().Get()
h_ap = cam.GetHorizontalApertureAttr().Get()
v_ap = cam.GetVerticalApertureAttr().Get()

fx = RENDER_W * focal / h_ap
fy = RENDER_H * focal / v_ap

cam_pos = Gf.Vec3d(UsdGeom.Xformable(cam_prim).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default()).ExtractTranslation())
cube_pos = Gf.Vec3d(UsdGeom.Xformable(
    stage.GetPrimAtPath(CUBE_PATH)).ComputeLocalToWorldTransform(
    Usd.TimeCode.Default()).ExtractTranslation())
dist = (cam_pos - cube_pos).GetLength()

tag_px = fx * TAG_SIZE_M / dist
view_w = RENDER_W * dist / fx
view_h = RENDER_H * dist / fy

print("")
print(f"[cam] projection {cam.GetProjectionAttr().Get()}")
print(f"[cam] focal {focal}  hAp {h_ap:.4f}  vAp {v_ap:.4f}")
print(f"[cam] fx {fx:.1f}  fy {fy:.1f}   "
      f"{'(square pixels OK)' if abs(fx - fy) / max(fx, fy) < 0.02 else '(!! STILL STRETCHED)'}")
print(f"[cam] distance to cube {dist:.4f} m")
print(f"[cam] view covers {view_w:.3f} x {view_h:.3f} m at the cube")
print("")
print(f"[cam] >>> tag will be {tag_px:.1f} px across ({tag_px / 8.0:.1f} per cell)")
print("[cam] >>> " + ("GOOD" if tag_px >= 40 else
                      "OK, tight" if tag_px >= 30 else "TOO SMALL"))
