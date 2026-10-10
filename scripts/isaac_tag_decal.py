"""Isaac Sim Script Editor — stick an AprilTag decal on a cube's top (+Z) face.

Paste into Window > Script Editor with the stage loaded, edit the CONFIG block,
and run. Creates ``<CUBE_PATH>/tag``: a flat UV-mapped quad textured with the
tag as *both* diffuse and emissive, so scene lighting and shadows can't wash out
the corners solvePnP depends on. Visual only — no collider, no rigid body — so
it cannot interfere with grasping or physics.

Safe to re-run: it deletes and rebuilds the same prims, so tweak and re-run
freely.

The quad is sized as a FRACTION of the cube's own size attribute rather than in
absolute metres. Both live in the cube's object space, so any scale on the cube
(or any ancestor) cancels out — otherwise a cube built as size=2 scaled by
0.025 would silently shrink the decal by 40x.

Prints ``tag_size_m`` at the end. That is the number AprilTagDetector wants —
the BLACK square's edge, which is *not* the quad edge. See QUIET_RATIO.
"""
import os

import omni.usd
from pxr import Usd, UsdGeom, UsdShade, Sdf

# ============================== CONFIG ==============================
CUBE_PATH     = "/World/Cube"   # exact path — copy it from the Stage panel
TAG_PNG       = os.path.expanduser(
    "~/isaacsim_ros_ws/apriltags/apriltag_36h11_id0.png")
QUAD_FRACTION = 0.96            # quad edge as a fraction of the cube edge
LIFT_FRACTION = 0.02            # lift above the face, same fractional units
# ====================================================================

# Measured from the PNGs in apriltags/: each is 1040x1040 px and the black
# pattern occupies rows/cols 120-919, i.e. 800 px. The remaining 120 px border
# is the white quiet zone the detector needs to find the tag's outer edge — so
# it must stay in the texture, but it is NOT part of the tag's metric size.
QUIET_RATIO = 800.0 / 1040.0    # = 0.769 = 10/13

stage = omni.usd.get_context().get_stage()
cube = stage.GetPrimAtPath(CUBE_PATH)
if not cube:
    raise RuntimeError(f"no prim at {CUBE_PATH} — check the Stage panel")

tag_path = CUBE_PATH + "/tag"
mat_path = CUBE_PATH + "/TagMat"
# Clear first, so the measurements below see the bare cube on a re-run.
for p in (tag_path, mat_path):
    if stage.GetPrimAtPath(p):
        stage.RemovePrim(p)

# ---- the cube's extent in its OWN object space (see module docstring) ----
if cube.IsA(UsdGeom.Cube):
    edge_local = float(UsdGeom.Cube(cube).GetSizeAttr().Get())
else:
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeUntransformedBound(cube).ComputeAlignedRange()
    edge_local = float(rng.GetSize()[2])
    print(f"[tag] {CUBE_PATH} is not a UsdGeom.Cube — using its untransformed "
          f"bbox z-extent: {edge_local:.5f}")

# ---- and its TRUE size in world metres, as an independent cross-check ----
# Only valid while the cube is axis-aligned (it is, sitting flat on the belt).
wcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
wsize = wcache.ComputeWorldBound(cube).ComputeAlignedRange().GetSize()
print(f"[tag] cube world size = ({wsize[0]:.4f}, {wsize[1]:.4f}, "
      f"{wsize[2]:.4f}) m   <-- confirm this is the 5 cm cube you think it is")

# ---- material: tag as diffuse AND emissive, fully matte ----
# Emissive so the pattern reads crisply regardless of lighting; roughness 0.9
# and metallic 0 so there is no specular highlight glaring over the corners.
mat = UsdShade.Material.Define(stage, mat_path)
shader = UsdShade.Shader.Define(stage, mat_path + "/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
st = UsdShade.Shader.Define(stage, mat_path + "/ST")
st.CreateIdAttr("UsdPrimvarReader_float2")
st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
tex = UsdShade.Shader.Define(stage, mat_path + "/Tex")
tex.CreateIdAttr("UsdUVTexture")
# Authored RELATIVE to the stage file when it is saved on disk, so the stage
# still finds the texture under another user or clone location. An absolute
# path here is what broke portability before (2026-10-11).
root_file = stage.GetRootLayer().realPath
tag_asset = (os.path.relpath(TAG_PNG, os.path.dirname(root_file))
             if root_file else TAG_PNG)
tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tag_asset)
tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
    st.ConnectableAPI(), "result")
rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

# ---- the decal quad ----
# Wound counter-clockwise as seen from +Z so the surface normal points UP, at
# the camera. UVs are listed in the SAME order as the points (both start at the
# -X/-Y corner and go counter-clockwise); a consistent order is what keeps the
# texture un-mirrored, and a mirrored 36h11 tag will not decode at all.
h = edge_local * QUAD_FRACTION / 2.0
z = edge_local / 2.0 + edge_local * LIFT_FRACTION
mesh = UsdGeom.Mesh.Define(stage, tag_path)
mesh.CreatePointsAttr([(-h, -h, z), (h, -h, z), (h, h, z), (-h, h, z)])
mesh.CreateFaceVertexCountsAttr([4])
mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
mesh.CreateNormalsAttr([(0, 0, 1)] * 4)
mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
mesh.CreateExtentAttr([(-h, -h, z), (h, h, z)])
# Belt-and-braces: if the winding ends up reversed the quad still renders
# rather than showing as an invisible back-face.
mesh.CreateDoubleSidedAttr(True)
UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
    "st", Sdf.ValueTypeNames.TexCoord2fArray,
    UsdGeom.Tokens.faceVarying).Set([(0, 0), (1, 0), (1, 1), (0, 1)])
UsdShade.MaterialBindingAPI(mesh).Bind(mat)

# ---- the numbers the perception node needs ----
quad_world = wsize[0] * QUAD_FRACTION
print(f"[tag] decal created at {tag_path}")
print(f"[tag] quad edge     = {quad_world:.5f} m (full PNG, quiet zone included)")
print(f"[tag] >>> tag_size_m = {quad_world * QUIET_RATIO:.5f} m  "
      f"<-- pass THIS to AprilTagDetector, not the quad edge")
print(f"[tag] tag plane sits {wsize[2] / 2.0 + wsize[2] * LIFT_FRACTION:.5f} m "
      f"above the cube centre")
