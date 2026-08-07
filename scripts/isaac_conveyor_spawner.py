"""Isaac Sim Script Editor — conveyor object spawner with AprilTags.

Paste into Window > Script Editor (with the stage loaded) and run. It spawns a
tagged cube at the belt start on a timer; the conveyor's PhysX surface velocity
carries each cube into the pick zone. Cubes past DESPAWN_Y are removed. Stop the
loop by running:  _spawner_sub.unsubscribe()

Targets Isaac Sim 6.0 (pxr USD + PhysX). It could NOT be tested against your
Windows Isaac Sim from here, so adjust the CONFIG block to your scene and paste
back any errors — the pxr calls used are the stable, version-robust ones.
"""
import time

import omni.usd
import omni.kit.app
from pxr import (Usd, UsdGeom, UsdShade, Sdf, Gf, UsdPhysics)

# ==================== CONFIG (edit to match your scene) ====================
PARENT_PATH  = "/World/ConveyorObjects"
SPAWN_POS    = Gf.Vec3d(0.90, -0.45, 0.90)  # world metres: just above the belt start
CUBE_SIZE    = 0.05                          # cube edge (m)
TAG_SIZE     = 0.045                         # black-square edge on the top plate (m)
SPAWN_PERIOD = 4.0                           # seconds between spawns
MAX_LIVE     = 6                             # cap on simultaneous cubes
DESPAWN_Y    = 0.60                          # remove once world-Y passes the belt end
# Copy a tag PNG to a Windows path, OR use the WSL UNC path shown below.
TAG_TEXTURE  = r"\\wsl.localhost\Ubuntu\home\aravindsuresh16\isaac_sim_ros\apriltags\apriltag_36h11_id0.png"
# ===========================================================================

stage = omni.usd.get_context().get_stage()
_SHARED_MAT_PATH = PARENT_PATH + "/TagMat"


def _ensure_parent():
    if not stage.GetPrimAtPath(PARENT_PATH):
        UsdGeom.Xform.Define(stage, PARENT_PATH)


def _ensure_tag_material():
    """One shared UsdPreviewSurface that shows the tag as both diffuse AND
    emissive, so it reads crisply regardless of scene lighting (key for robust
    detection). Matte (high roughness) to avoid specular glare."""
    if stage.GetPrimAtPath(_SHARED_MAT_PATH):
        return UsdShade.Material.Get(stage, _SHARED_MAT_PATH)
    mat = UsdShade.Material.Define(stage, _SHARED_MAT_PATH)
    shader = UsdShade.Shader.Define(stage, _SHARED_MAT_PATH + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    st = UsdShade.Shader.Define(stage, _SHARED_MAT_PATH + "/ST")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, _SHARED_MAT_PATH + "/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(TAG_TEXTURE)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        st.ConnectableAPI(), "result")
    rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _add_tag_plane(obj_path, mat):
    """Flat UV-mapped quad on the cube's +Z face, textured with the tag."""
    mesh = UsdGeom.Mesh.Define(stage, obj_path + "/tag")
    h = TAG_SIZE / 2.0
    z = CUBE_SIZE / 2.0 + 0.001
    mesh.CreatePointsAttr([(-h, -h, z), (h, -h, z), (h, h, z), (-h, h, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([(0, 0, 1)] * 4)
    pv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
    pv.Set([(0, 0), (1, 0), (1, 1), (0, 0)][:3] + [(0, 1)])  # TL,TR,BR,BL
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)


_counter = {"n": 0}


def spawn_one():
    _ensure_parent()
    mat = _ensure_tag_material()
    i = _counter["n"]; _counter["n"] += 1
    path = f"{PARENT_PATH}/obj_{i:03d}"
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(SPAWN_POS)
    # dynamic rigid body: RigidBody on the Xform, Collision on the cube geom
    body = UsdGeom.Cube.Define(stage, path + "/body")
    body.CreateSizeAttr(CUBE_SIZE)
    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr(0.05)
    _add_tag_plane(path, mat)
    return path


def _despawn_offbelt():
    parent = stage.GetPrimAtPath(PARENT_PATH)
    if not parent:
        return
    for child in list(parent.GetChildren()):
        if not child.IsA(UsdGeom.Xform):
            continue
        m = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        if m.ExtractTranslation()[1] > DESPAWN_Y:
            stage.RemovePrim(child.GetPath())


# Periodic spawn on the app update stream (safe for stage edits; wall-clock dt
# so we don't depend on the event payload schema).
_state = {"last": time.time(), "acc": 0.0}


def _on_update(_e):
    now = time.time()
    _state["acc"] += now - _state["last"]
    _state["last"] = now
    if _state["acc"] >= SPAWN_PERIOD:
        _state["acc"] = 0.0
        parent = stage.GetPrimAtPath(PARENT_PATH)
        live = len(parent.GetChildren()) - 1 if parent else 0  # minus shared mat
        if live < MAX_LIVE:
            spawn_one()
        _despawn_offbelt()


_spawner_sub = omni.kit.app.get_app().get_update_event_stream() \
    .create_subscription_to_pop(_on_update, name="conveyor_spawner")
print("Conveyor spawner running. Stop with:  _spawner_sub.unsubscribe()")
