# UR5e Conveyor Pick — Project Plan

**Goal:** a UR5e + Robotiq 2F-85 in Isaac Sim, driven over ROS 2 Jazzy with MoveIt2 for planning,
picks an AprilTag-marked cube off a conveyor and places it in a bin. End target is a *dynamic
intercept* pick (moving cube); building up from a fixed-pose pick first.

**Environment:** dual-boot Ubuntu 24.04. Isaac Sim 6.0.1 (`~/isaacsim`) and ROS 2 Jazzy
(`/opt/ros/jazzy`) both run natively on the same machine. Workspace: `~/isaacsim_ros_ws`,
package `ur5e_isaac_control`.

> **Why dual-boot:** an earlier iteration ran Isaac Sim on Windows talking to ROS 2 in WSL2, where
> mirrored networking broke UDP-multicast DDS discovery and the Fast-DDS workaround in turn broke
> MoveIt's service/action traffic. Both sides on one native install removes the problem entirely —
> **confirmed 2026-08-10**, zero DDS configuration. If `ros2 topic list` or MoveIt behave oddly, it
> is a *new* problem, not a recurrence of that bug. History: [ARCHIVE_WSL2_WINDOWS_ISSUES.md](ARCHIVE_WSL2_WINDOWS_ISSUES.md).

## How I'd like you to work with me

- **Concept before commands.** Explain what a step does and why, before running it.
- **One step at a time.** Stop and check I've understood before moving on.
- **Be explicit about verified vs. assumed.** "I haven't confirmed this" beats a confident guess.
- **Correct me plainly** if I state something wrong.

I have hands-on Isaac Sim experience; started MoveIt/ROS 2 at zero.

## Companion docs

| File | What it is |
|---|---|
| [EXPLAINED.md](EXPLAINED.md) | How the pieces work and why, concept-first |
| [ISSUES.md](ISSUES.md) | Known traps, settled decisions, resolved bugs — **read before debugging** |
| [ARCHIVE_WSL2_WINDOWS_ISSUES.md](ARCHIVE_WSL2_WINDOWS_ISSUES.md) | Retired WSL2/Windows setup |

## Workspace map

| File | What it is |
|---|---|
| `src/ur5e_isaac_control/launch/moveit.launch.py` | Brings up `robot_state_publisher`, `trajectory_bridge`, `move_group`, `rviz2` |
| `.../launch/pick_demo.launch.py` | **The whole stack in one command.** *Includes* `moveit.launch.py` (not a copy) and adds `cube_pose_publisher` + `pick_orchestrator`. `auto_start` defaults to **false** — see M4 |
| `.../ur5e_isaac_control/trajectory_bridge.py` | Custom `FollowJointTrajectory` action server → `/joint_command` |
| `.../joint_command_publisher.py` | Scripted joint streamer (proven working) |
| `.../joint_controller_gui.py` | Tk slider panel for jogging joints |
| `.../kinematics.py` | ikpy-based FK/IK, independent of MoveIt (ikpy installed user-local, `--no-deps`) |
| `.../apriltag_pose.py` | OpenCV AprilTag 6-DOF pose estimation |
| `.../cube_pose_publisher.py` | **The perception node.** `/rgb` + `/camera_info` → `/cube_pose` (PoseStamped in `base_link`), `/cube_marker`, static `base_link → sim_camera` TF |
| `.../pick_orchestrator.py` | **The M4 state machine.** `/cube_pose` → MoveGroup action → gripper. Runs with an **empty planning scene by design** — read its docstring before adding M3 |
| `src/ur5e_isaac_control/urdf/ur5e.urdf` | Arm-only description, verified against the Isaac articulation |
| `.../urdf/ur5e_gripper.urdf.xacro` | **Composed** description: `ur5e.urdf` verbatim + 2F-85 geometry + `grasp_tcp`. This is what `moveit.launch.py` loads |
| `.../config/ur5e_gripper.srdf` | SRDF in use: `ur_manipulator` tipped at `grasp_tcp` + collision matrix. Copy of the Setup Assistant output |
| `src/ur5e_gripper_moveit_config/` | MSA output. Kept as the SRDF regeneration source; its launch/controller files are deliberately **not** used |
| `scripts/isaac_conveyor_spawner.py` | Spawns cubes on the conveyor (⚠️ two stale lines — see ISSUES) |
| `scripts/run_setup_assistant.sh` | Launches MSA with the VSCode-snap env stripped |
| `scripts/isaac_joints_dump.py` | Prints every USD physics joint (parent/child + local frames). Run inside Isaac Sim |
| `scripts/isaac_gripper_measure.py` | Gripper prim transforms/bboxes. ⚠️ cross-link transforms unreliable — see ISSUES |
| `scripts/isaac_tag_decal.py` | Sticks an AprilTag decal on a cube's +Z face; sizes it as a *fraction* of the cube so prim scale cancels, and prints `tag_size_m` |
| `scripts/isaac_camera_setup.py` | Sets camera projection + lens atomically, reads back, predicts the tag's pixel size. Forces `perspective` |
| `scripts/isaac_camera_check.py` | Read-only version. Refuses to run on an orthographic camera |
| `scripts/isaac_camera_extrinsic.py` | Computes `base_link → sim_camera`, folding in the USD→ROS optical flip. **Re-run whenever the camera *or* robot base moves** |
| `scripts/isaac_rebake_fixed_joint.py` | Re-bakes a `FixedJoint` to the bodies' current placement |
| `scripts/isaac_scene_measure.py` | Bounding boxes of the conveyor/table/bin in `base_link`. Output in `scene_measure.txt` (2026-08-11) — the measurement that showed naive M3 blocks M4 |
| `apriltags/` | Print-ready 36h11 tags, ids 0–3. 1040×1040 px, **inner 800 px is the black square** (ratio 10/13) |

## Target architecture

```
Isaac Sim (Ubuntu, native)                    ROS 2 Jazzy (Ubuntu, native, same box)
---------------------------                    --------------------------------------
UR5e + Robotiq 2F-85          /joint_states     move_group (ur_moveit_config)
articulation(s)          <=================>        ^
  ROS2 Publish/Subscribe    (DDS, standard           | plan requests
  Joint State (built)        discovery)               |
        ^                                     pick_orchestrator (written, unrun)
        |  /joint_command, /gripper_command     - state machine
        |  (FollowJointTrajectory bridge, built)  - talks to move_group via the
        |                                           MoveGroup action directly
        |                                           (no moveit_py on this box)
        +--- trajectory_bridge node (built) <----+
                                                       |
                                              conveyor_kinematics node (new)
                                                - spawn time + belt speed
                                                -> predicted cube pose/time
```

RViz + the MoveIt "Motion Planning" plugin sits alongside `move_group` — not required at runtime
once the orchestrator is scripted, but essential while building each milestone.

## Status

| Piece | State |
|---|---|
| Isaac ↔ ROS2 joint bridge (Action Graph, `/joint_states`, `/joint_command`) | ✅ verified |
| `joint_command_publisher`, `joint_controller_gui` | ✅ verified |
| `kinematics.py` (ikpy FK/IK) | ✅ validated — 200-pose IK round-trip, 0.0000 mm (re-verified here 2026-09-04) |
| `apriltag_pose.py` (36h11 + solvePnP) | ✅ verified live 2026-08-11 off the Isaac camera |
| Gripper (2F-85) ROS2 wiring — **M0** | ✅ commandable (`finger_joint`: 0.0 open → ~0.6–0.8 closed) |
| MoveIt bring-up (`moveit.launch.py`) — **M1** | ✅ verified 2026-08-10 — RViz Plan succeeds (5–25 ms typical) |
| Sim-time alignment (`/clock` + `use_sim_time`) | ✅ verified 2026-08-10 — prerequisite for M2 |
| `trajectory_bridge` (FollowJointTrajectory → `/joint_command`) — **M2** | ✅ verified live 2026-08-10 — `SUCCEEDED`, worst joint error 0.0000–0.0001 rad |
| URDF gripper geometry + `grasp_tcp` — **M1b** | ✅ verified 2026-08-10 — plan+execute across the reachable workspace |
| Isaac scene: camera + tagged cube + bin | ✅ verified 2026-08-11 — tag detected live at 56.8 px |
| Perception node (`cube_pose_publisher`) | ✅ verified 2026-08-11 — `/cube_pose` at ~17 Hz, **0.2 mm X, 0.3 mm Y, exact Z** vs USD truth. **Re-verified live 2026-09-12**: 600 detections, zero dropouts, cube at `(-0.3649, +0.6161, -0.2040)` — within 0.3 mm of the 2026-08-11 record, so the scene has not moved and the extrinsic still holds |
| Planning-scene collision objects — **M3** | ⏸️ **deliberately deferred** — the naive version *blocks* M4, see below |
| Fixed-point pick-and-place (`pick_orchestrator`) — **M4** | 🟡 **written 2026-09-04, never yet run** — code complete, unproven |
| One-command bring-up (`pick_demo.launch.py`) | 🟡 written 2026-09-12, parses clean, not yet run |
| Dynamic intercept (`conveyor_kinematics`) — **M5** | ⬜ not started |
| Closed-loop tracking via `moveit_servo` (stretch) — **M6** | ⬜ not started |

**Immediate next step: run the M4 pick.** Everything it needs is built and verified — perception
re-confirmed live on 2026-09-12, the orchestrator written, the launch file staged. Nothing has been
executed yet. Two commands:

```bash
ros2 launch ur5e_isaac_control pick_demo.launch.py        # comes up idle
ros2 service call /pick_orchestrator/pick_once std_srvs/srv/Trigger
```

> **`auto_start` is false in the launch file although the node's own default is true.** The node
> starts its sequence 2 s (sim time) after coming up; launching the whole stack at once means the
> arm moves before you can confirm RViz looks sane and `/cube_pose` is flowing, and `move_group` is
> not always ready inside 2 s. Bring it up idle, check, then fire. Pass `auto_start:=true` once the
> sequence is trusted.

**This one run tests three things at once:**

1. **M4 itself** — the full plan → execute → grasp → place chain.
2. **The USD↔URDF `base_link` yaw question** (see ISSUES.md). Both are axis-aligned conventions, so
   the offset can only be 0° or a multiple of 90° — never a subtle few degrees. If they disagree the
   arm reaches somewhere *visibly* wrong. This is the cheap version of the check; the two-sided FK
   measurement is only worth building if the pick fails in a way that implicates the frame.
3. **`grasp_z_offset`**, currently `0.0` — the dial most likely to need tuning. `grasp_tcp`'s
   0.11181 m is the fingertip *link origin*, not the pad contact face, so the fingers may close
   slightly high or low. Fingers closing on air, or the wrist driving into the belt, points here.

## Scene reference numbers (as built 2026-08-11)

Post-move values. **Anything derived from these needs re-deriving if the scene moves again** — in
particular re-run `isaac_camera_extrinsic.py`.

| Thing | Value |
|---|---|
| Cube | `/World/Cube`, **0.05 m**, RigidBody on the belt. World `(14.39997, 2.35000, 1.80553)` |
| Tag decal | `/World/Cube/tag`, quad edge 0.048 m (= 0.96 × cube edge) |
| **`tag_size_m`** | **0.03692 m** — the *black square*, = quad × 800/1040. Feed this to `AprilTagDetector` |
| Tag plane | 0.026 m above the cube centre (0.025 + 1 mm z-fight lift) |
| Robot base | `/World/ur5e/base_link`, world `(14.76516, 1.73420, 2.00951)`, no rotation relative to world |
| Camera | `/World/Camera`, **perspective**, focalLength 80, hAp 27.8254, vAp 15.6518 |
| Intrinsics | fx = fy = **3680.09**, cx 640, cy 360, zero distortion, 1280×720 |
| Camera pose | world `(14.2222, 2.5796, 4.1773)`, pointing **straight down** |
| **Extrinsic** | `base_link → sim_camera` (optical): xyz `(-0.542991, +0.845386, +2.167835)`, quat xyzw `(0.707112, -0.707101, 0, -0.000011)` |
| **`tag_z_base`** | **-0.177978 m** — tag plane height in `base_link`. The node's depth reference |
| Cube in `base_link` | `(-0.3652, +0.6158, -0.2040)`, **0.745 m from base** (UR5e reach 0.850 m) |
| View covered | 0.85 × 0.48 m; camera→tag 2.364 m |
| Topics | `/rgb` (rgb8, ~20 Hz) and `/camera_info`, both `frame_id: sim_camera`, sim-time stamped |

Camera Action Graph is kept **separate** from the joint/clock graph so it can be disabled on its
own (8 GB VRAM): `On Playback Tick` → `Isaac Create Render Product` (`/World/Camera`, 1280×720) →
`ROS2 Camera Helper` (`rgb`) **and** `ROS2 Camera Info Helper` (`camera_info`), each taking both
`execOut` and `renderProductPath`.

**Belt velocity is currently 0** so the cube stays put for the fixed-pose pick. M5 turns it back on.

### Perception accuracy, in one paragraph

Raw vision is accurate laterally but reads ~2% far in depth (mipmap blur shrinks the tag ~1.1 px at
14:1 minification, and depth comes entirely from apparent size). Because lateral position is
`(u − cx)/fx · Z`, that depth bias **leaks into X and Y** in proportion to off-axis angle — at 7°
off-axis it was +4.5 mm X, −5.1 mm Y. The fix is a **ray–plane intersection** (the direction to the
tag is reliable, only the distance along it is biased), which corrects all three axes at once and
does not drift as the cube moves — unlike shrinking `tag_size_m`, which only absorbs the bias at
one distance. Implemented in `cube_pose_publisher`: **X +0.2 mm, Y +0.3 mm, Z exact (asserted), Z
jitter 0** over 200 consecutive detections. Full derivation in [EXPLAINED.md](EXPLAINED.md) §11.

> ⚠️ **What this does *not* prove:** the extrinsic and the "truth" both come from USD's `base_link`,
> so a yaw offset between USD's `base_link` and the URDF's would leave both wrong together and this
> check would still pass. **Settle before M4** — see ISSUES.md.

## Milestones

**M0 — Gripper ROS2 wiring.** ✅ The 2F-85 is controllable via `/joint_command`'s `finger_joint`.

**M1 — MoveIt bring-up.** ✅ **2026-08-10.** `moveit.launch.py` runs `robot_state_publisher` with
our URDF and parameterizes `move_group` from `ur_moveit_config`'s SRDF/kinematics/OMPL/joint-limits
YAML — UR's own launch file can't be used as-is (EXPLAINED.md §3). Planning group `ur_manipulator`
(`base_link` → `tool0`); the SRDF ships named goals `home`, `up`, `test_configuration`, the fastest
way to get a guaranteed-valid goal when debugging.

**M1b — URDF gripper geometry + `grasp_tcp`.** ✅ **2026-08-10.** `ur5e_gripper.urdf.xacro`
includes `ur5e.urdf` verbatim and bolts the 2F-85 to `tool0` as a single **fixed** link carrying
the nine vendor collision meshes; the SRDF tips `ur_manipulator` at `grasp_tcp`. Non-obvious
decisions:

- **Meshes from `ros-jazzy-robotiq-description`.** Nothing ships a combined UR+gripper description,
  so composition is unavoidable.
- **One merged fixed link, not the vendor macro.** Isaac publishes 6 gripper joints under
  *different names* than `robotiq_description` uses. Movable joints under vendor names would stay
  permanently unpublished, so MoveIt's `CurrentStateMonitor` would never see a complete state and
  every Execute would abort — the same failure the `/clock` fix cured.
- **Geometry modelled OPEN (knuckle θ=0), `grasp_tcp` at CLOSED (θ=0.8, z=0.11181 m from the
  gripper base).** Open is the wider envelope so collision checking errs safe; the closed fingertip
  is where a grasped object actually sits. Cross-check: the same FK reproduces the published 85 mm
  stroke to 84.8 mm.
- **Mount is identity** — `tool0` (+Z out of the flange) and `robotiq_85_base_link` (+Z approach)
  already agree.
- `grasp_tcp`'s 0.11181 m is the fingertip **link origin**, not the pad contact face, so
  `kinematics.py`'s `tcp_z_offset` should start there and be trimmed once M4 can test a real grasp.

**M2 — The FollowJointTrajectory bridge.** ✅ **Verified live 2026-08-10** (`trajectory_bridge.py`).
Advertises `/scaled_joint_trajectory_controller/follow_joint_trajectory` (the exact name
`ur_moveit_config`'s `moveit_controllers.yaml` expects), consumes MoveIt's `JointTrajectory`, and
streams it to `/joint_command` at 60 Hz by sampling against sim time. Proof: RViz Execute moved the
arm — 49 points, 4.74 s, worst joint error 0.0010 rad, `SUCCEEDED`.

**M3 — Planning scene collision objects.** ⏸️ **Deliberately deferred, and M4 goes first.** This
was originally written as a prerequisite for M4. Measurement proved the opposite: done naively it
**blocks** M4.

`scripts/isaac_scene_measure.py` (2026-08-11) found the conveyor's bounding box spans
`x [-0.768, 3.235]`, `y [+0.165, +1.341]`, `z [-2.010, +0.301]` in `base_link` — and the cube at
`(-0.365, +0.616, -0.204)` sits **inside it on all three axes**. Register the conveyor as one box
from its AABB and *every* grasp pose is permanently in collision; nothing will ever plan.

Why skipping it is safe here:

- **Self-collision checking is unaffected** — it comes from the SRDF's collision matrix, not the
  world, so the arm still cannot fold into itself.
- **An empty world is the easiest case for OMPL.** Every collision object is one more way to fail,
  so an empty scene maximises the chance of a first successful pick.
- **In simulation, interpenetration costs nothing.** PhysX pushes through; no hardware at risk.

⚠️ **The consequence to accept:** the arm can and may plan straight through the conveyor, the table
and the bin. Fine in sim. **Do not port this to a real arm without doing M3 properly first.**

When M3 is eventually done properly, two things must be true: furniture geometry stops at the
**belt surface** rather than using the full AABB, and the bin is added before place poses are
trusted. Note also that `isaacsim.ros2.nodes` (v1.18.13, verified 2026-09-12) ships
**`ROS2PublishTransformTree`**, whose `parentPrim` can be set to `/World/ur5e/base_link` so furniture
poses arrive in ROS already in the robot's frame — a live alternative to transcribing numbers. It
publishes *poses*, not *geometry*, so box dimensions would still be authored by hand. Do not point
it at the robot articulation: `robot_state_publisher` already owns `base_link → *`, and two
publishers for one child frame corrupts the TF tree.

**M4 — Fixed-point pick-and-place.** 🟡 **Written 2026-09-04 (`pick_orchestrator.py`, 490 lines),
never yet run.** Sequence: `home → above cube → down to grasp → close → lift → above bin → release
→ retreat → home`. Proves the full chain before adding motion. Remember the tag is on the cube's
**top face** — `T_base_tag` is a plane 0.026 m above the cube centre, so grasp poses need that
offset or the gripper closes on air.

Decisions taken while writing it, each non-obvious:

- **Sends the `MoveGroup` action directly, not `MoveGroupInterface`.** This install has no
  `moveit_py` bindings (`import moveit` fails; only `moveit_msgs` is present), so the Python
  wrapper PLAN.md originally assumed does not exist. Posting `moveit_msgs/action/MoveGroup` to
  `/move_action` is what that wrapper does internally, and it keeps the node dependency-free.
- **Grasp orientation.** `grasp_tcp`'s +Z is the approach axis (inherited from `tool0`), and the
  cube is picked from directly above, so +Z must point along `base_link`'s −Z: a 180° rotation
  about X composed with the cube's yaw about base Z — `R = Rz(yaw) · Rx(π)`, i.e. quaternion
  `(x,y,z,w) = (cos(yaw/2), sin(yaw/2), 0, 0)`. At yaw = 0 that is `(1,0,0,0)`, a clean flip.
  `cube_pose_publisher` already folds yaw into ±45°, so the wrist never takes the long way round.
- **The cube pose is latched at pick time, then vision stops being consulted.** Once the gripper is
  over the cube it occludes the tag, and once grasped the cube's pose is defined by the gripper, not
  by vision. The sequence checks one pose for freshness (`cube_pose_max_age_sec`, 5 s) and commits.
  M5 is where a live pose stream starts to matter.
- **Conservative defaults for a first run:** velocity and acceleration scaling at 0.1, approach and
  lift heights 0.15 m, and a `max_reach` 0.850 m preflight check (the cube is at 0.745 m).
- A `~/pick_once` `std_srvs/Trigger` service fires the sequence on demand, which is how
  `pick_demo.launch.py` drives it with `auto_start:=false`.

**M5 — Dynamic intercept (the actual target).** A `conveyor_kinematics` node computes the cube's
live pose and picks an intercept point + arrival time; the orchestrator plans once to that
predicted pose, timed to finish on arrival. A single MoveIt plan takes 50–200 ms+ — too slow to
re-plan every frame — so **predict-and-intercept** is the standard approach, not continuous
re-planning.

> Belt velocity is **not** in `isaac_conveyor_spawner.py` (it only defines `SPAWN_POS`, `DESPAWN_Y`
> and spawn timing) — cubes move via the conveyor prim's PhysX surface velocity on the USD stage.
> M5 must either read that off the prim, or derive speed empirically from observed motion (which
> also self-corrects for belt slip).

**M6 — Closed-loop tracking (stretch).** Only if M5's open-loop prediction proves insufficient:
feed the live pose stream into `moveit_servo` (already installed; `ur_moveit_config` ships
`config/ur_servo.yaml`) for continuous velocity-based tracking.

## Order of work

```
M1 ──┬── M1b ──┐
     │         ├── M2 ── M4 ── M5 ── M6
M0 ──┴─────────┘        │
                        └── M3  (off the critical path — see the milestone)
```

M0, M1, M1b and M2 are built **and verified live** (2026-08-10) — the whole
plan→execute→gripper-aware chain works. Perception is complete and verified live (2026-08-11,
re-confirmed 2026-09-12).

**M3 was moved off the critical path** once measurement showed the naive version makes every grasp
pose permanently in collision. M4 runs against an empty planning scene instead.

Next up: **run M4** — the code exists and has never been executed. Everything upstream of it is
verified, so a failure now points at the orchestrator, the frame convention, or `grasp_z_offset`,
not at perception or the bridge.

## Perception approach (reference)

Deliberately lightweight — 36h11 AprilTags + OpenCV `solvePnP`, no GPU inference, no external
models. **Eye-to-hand** (world-mounted camera) rather than eye-in-hand, chosen so `T_base_cam` is a
**constant** read once off the stage and so the camera keeps watching the belt while the arm is
busy — which M5 needs.

`cube_pose_publisher` subscribes `/rgb` + `/camera_info`, runs `apriltag_pose.py`, applies the
static extrinsic, corrects depth by ray–plane intersection, and publishes `/cube_pose`,
`/cube_marker` and the static TF. Everything is a ROS parameter, so a scene move means changing a
number, not editing code. Orientation is **yaw only** — roll/pitch forced to zero (the cube lies
flat; a straight-down view makes tilt degenerate), yaw folded into ±45° since a cube looks
identical every 90°.

The frame chain is three links, and the middle one is the one everyone forgets:

```
T_base_tag  =  T_base_camlink · T_camlink_optical · T_optical_tag
                    ↑                  ↑                 ↑
               read off USD     fixed ROS convention   from the detector
```

`apriltag_pose.py` returns `T_optical_tag` in the **OpenCV optical frame** (X right, Y down, Z out
of the lens); ROS's `camera_link` convention is X forward, Y left, Z up — a fixed 90°/90° rotation
apart (quaternion `(-0.5, 0.5, -0.5, 0.5)`). Omit it and the cube lands at a plausible distance in
entirely the wrong direction.
