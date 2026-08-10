# UR5e Conveyor Pick — Project Plan

**Goal:** a UR5e + Robotiq 2F-85 in Isaac Sim, driven over ROS 2 Jazzy with MoveIt2 for planning,
picks an AprilTag-marked cube off a conveyor and places it in a bin. End target is a *dynamic
intercept* pick (moving cube); building up from a fixed-pose pick first.

**Environment:** dual-boot Ubuntu 24.04. Isaac Sim 6.0.1 (`~/isaacsim`) and ROS 2 Jazzy
(`/opt/ros/jazzy`) both run natively on the same machine. Workspace: `~/isaacsim_ros_ws`,
package `ur5e_isaac_control` (ament_python).

> **Why dual-boot:** an earlier iteration ran Isaac Sim on Windows talking to ROS 2 in WSL2. WSL2's
> mirrored networking silently breaks UDP-multicast DDS discovery, which needed a hand-built
> Fast-DDS unicast-loopback profile to work around — and that profile in turn broke MoveIt's
> service/action traffic (planning fine, but RViz's plan-result fetch hung forever). Moving both
> sides onto one native Ubuntu install removes the host boundary and the workaround entirely; DDS
> discovery just works — **confirmed 2026-08-10**: `/joint_states`, `/joint_command`, `/clock`, and
> MoveIt's full service/action traffic (plan *and* execute) all work with zero DDS configuration.
> `~/.bashrc` has no `FASTRTPS_DEFAULT_PROFILES_FILE`, and the `rviz_env` var-stripping that worked
> around the old profile has been deleted from `launch/moveit.launch.py`. If `ros2 topic list` or
> MoveIt behave oddly, it is a *new* problem, not a recurrence of the DDS bug.

## How I'd like you to work with me

- **Concept before commands.** Explain what a step does and why, before running it.
- **One step at a time.** Stop and check I've understood before moving on.
- **Be explicit about verified vs. assumed.** "I haven't confirmed this" beats a confident guess.
- **Correct me plainly** if I state something wrong.

I have hands-on Isaac Sim experience; started MoveIt/ROS 2 at zero.

## Workspace map

| File | What it is |
|---|---|
| `docs/PLAN.md` | This file — goals, status, milestones |
| `docs/EXPLAINED.md` | How the pieces work and why, concept-first |
| `src/ur5e_isaac_control/launch/moveit.launch.py` | Brings up `robot_state_publisher`, `trajectory_bridge`, `move_group`, `rviz2` |
| `src/ur5e_isaac_control/ur5e_isaac_control/trajectory_bridge.py` | Custom `FollowJointTrajectory` action server → `/joint_command` |
| `src/ur5e_isaac_control/ur5e_isaac_control/joint_command_publisher.py` | Scripted joint streamer (proven working) |
| `src/ur5e_isaac_control/ur5e_isaac_control/joint_controller_gui.py` | Tk slider panel for jogging joints |
| `src/ur5e_isaac_control/ur5e_isaac_control/kinematics.py` | ikpy-based FK/IK (independent of MoveIt) |
| `src/ur5e_isaac_control/ur5e_isaac_control/apriltag_pose.py` | OpenCV AprilTag 6-DOF pose estimation |
| `src/ur5e_isaac_control/urdf/ur5e.urdf` | Arm-only description (robot name `ur5e`), verified against the Isaac Sim articulation |
| `src/ur5e_isaac_control/urdf/ur5e_gripper.urdf.xacro` | **Composed** description: includes `ur5e.urdf` verbatim + Robotiq 2F-85 geometry + `grasp_tcp`. This is what `moveit.launch.py` loads |
| `src/ur5e_isaac_control/config/ur5e_gripper.srdf` | SRDF in use: `ur_manipulator` tipped at `grasp_tcp` + collision matrix. A copy of the Setup Assistant output — see its header to refresh |
| `src/ur5e_gripper_moveit_config/` | MoveIt Setup Assistant output. Kept as the SRDF regeneration source; its launch/controller files are deliberately **not** used |
| `scripts/isaac_conveyor_spawner.py` | Spawns cubes on the conveyor in Isaac Sim |
| `scripts/run_setup_assistant.sh` | Launches MSA with the VSCode-snap env stripped (it crashes otherwise) |
| `scripts/isaac_joints_dump.py` | Prints every USD physics joint (parent/child + local frames). Run inside Isaac Sim |
| `scripts/isaac_gripper_measure.py` | Prints gripper prim transforms/bboxes. ⚠️ its cross-articulation transforms are unreliable — see Known issues |
| `apriltags/` | Print-ready 36h11 tags, ids 0–3 |

## Target architecture

```
Isaac Sim (Ubuntu, native)                    ROS 2 Jazzy (Ubuntu, native, same box)
---------------------------                    --------------------------------------
UR5e + Robotiq 2F-85          /joint_states     move_group (ur_moveit_config)
articulation(s)          <=================>        ^
  ROS2 Publish/Subscribe    (DDS, standard           | plan requests
  Joint State (built)        discovery)               |
        ^                                     pick_orchestrator node (new)
        |  /joint_command, /gripper_command     - state machine
        |  (FollowJointTrajectory bridge, built)  - talks to move_group via
        |                                           MoveGroupInterface (Python)
        +--- trajectory_bridge node (built) <----+
                                                       |
                                              conveyor_kinematics node (new)
                                                - spawn time + belt speed
                                                -> predicted cube pose/time
                                                (later: swap in apriltag_pose.py
                                                 for real perception)
```

RViz + the MoveIt "Motion Planning" plugin sits alongside `move_group` for visualizing/debugging
plans — not required at runtime once the orchestrator is scripted, but essential while building
each milestone.

## Status

| Piece | State |
|---|---|
| Isaac Sim ↔ ROS2 joint bridge (Action Graph, `/joint_states`, `/joint_command`) | ✅ built & verified — see [EXPLAINED.md](EXPLAINED.md) |
| `joint_command_publisher` (scripted moves, demo sweep, pick-place mimic) | ✅ built & verified |
| `joint_controller_gui` (Tk slider jog panel) | ✅ built & verified |
| `kinematics.py` (ikpy FK/IK) | ✅ built & validated — 200-pose IK round-trip, 0.0000 mm error |
| `apriltag_pose.py` (36h11 detection + solvePnP) | ✅ built & validated — 50-pose test, 0.0002 mm error |
| Gripper (2F-85) ROS2 wiring — **M0** | ✅ commandable (`finger_joint`: 0.0 open → ~0.6–0.8 closed) |
| MoveIt bring-up (`moveit.launch.py`) — **M1** | ✅ built & **verified natively 2026-08-10** — RViz Plan succeeds (5–25 ms typical) |
| Sim-time alignment (`/clock` + `use_sim_time`) | ✅ built & verified 2026-08-10 — prerequisite for M2, see Known issues |
| `trajectory_bridge` (FollowJointTrajectory → `/joint_command`) — **M2** | ✅ built & **verified live 2026-08-10** — trajectories execute onto the Isaac Sim arm, `SUCCEEDED`, worst joint error 0.0000–0.0001 rad after the sim-clock fix |
| URDF gripper geometry + `grasp_tcp` frame — **M1b** | ✅ **done & verified 2026-08-10** — 2F-85 collision geometry on `tool0`, `ur_manipulator` tipped at `grasp_tcp`, plan+execute verified across the reachable workspace |
| Isaac Sim scene: camera + tagged cube + bin — perception scene setup | ⬜ not started |
| Perception node (subscribe `/rgb`, publish object pose in base frame) | ⬜ not started |
| Planning-scene collision objects (conveyor, table) — **M3** | ⬜ not started |
| Fixed-point pick-and-place (`pick_orchestrator`) — **M4** | ⬜ not started |
| Dynamic intercept (`conveyor_kinematics`, predict-and-intercept) — **M5** | ⬜ not started |
| Closed-loop tracking via `moveit_servo` (stretch) — **M6** | ⬜ not started |

**Immediate next step:** **M3** — register the conveyor, table, and bin as planning-scene collision
objects. M0/M1/M1b/M2 are all verified live (2026-08-10): MoveIt plans to the gripper's fingertips
and executes onto the Isaac Sim arm with essentially zero tracking error. The arm currently plans
through the furniture because the planning scene is empty (`world.collision_objects = []`), which
is the last thing between here and a real pick.

> Useful measurement already in hand for M3: `/World/RobotTable` exists on the stage, and the
> arm's base is fixed to it at `xyz=(-0.212666, +0.117802, +0.583556)` (from
> `/World/ur5e/base_link/FixedJoint`). `scripts/isaac_joints_dump.py` will re-print this, and the
> same approach gets the conveyor and bin.

The perception scene setup can proceed in parallel since it touches only the Isaac Sim stage.

## Milestones

**M0 — Gripper ROS2 wiring.** ✅ Done. The 2F-85 is controllable via `/joint_command`'s
`finger_joint`.

**M1 — MoveIt bring-up.** ✅ **Done and verified 2026-08-10.** Custom launch file
(`moveit.launch.py`) runs `robot_state_publisher` with our URDF, parameterizing `move_group` from
`ur_moveit_config`'s SRDF/kinematics/OMPL/joint-limits YAML — `ur_moveit_config`'s own launch file
can't be used as-is (see EXPLAINED.md §3). Proof achieved: RViz Motion Planning panel, goal set,
Plan succeeds, ghost arm animates. Planning group is `ur_manipulator` (`base_link` → `tool0`);
the SRDF also ships named joint-space goals `home`, `up`, `test_configuration`, which are the
fastest way to get a guaranteed-valid goal when debugging.

> Gotcha worth knowing: a *large* interactive-marker drag often fails with OMPL burning its full
> 5 s budget then `FAILURE` — the pose is unreachable, so there is no goal for the planner to grow
> toward. Small drags plan in milliseconds. When planning fails, first re-test with a named
> joint-space goal (`home`) to tell "goal was bad" apart from "the stack is broken".

**M1b — URDF gripper geometry + `grasp_tcp` frame.** ✅ **Done and verified 2026-08-10.**
`urdf/ur5e_gripper.urdf.xacro` includes `ur5e.urdf` verbatim and bolts the 2F-85 to `tool0` as a
single **fixed** link carrying the nine vendor collision meshes; `config/ur5e_gripper.srdf` (Setup
Assistant output) tips `ur_manipulator` at `grasp_tcp` and supplies the collision matrix. Verified:
plan+execute succeed across the reachable workspace, `ur_manipulator` still reports 6 DOF.

Key decisions and the reasons, since they are not obvious:

- **Meshes from `ros-jazzy-robotiq-description`.** Nothing ships a combined UR+gripper description:
  `ur_description` has zero gripper content and no apt package pairs them. Composition is
  unavoidable.
- **One merged fixed link, not the vendor macro.** Isaac Sim publishes 6 gripper joints under
  *different names* than `robotiq_description` uses (`finger_joint` / `left_inner_finger_joint` vs
  `robotiq_85_left_knuckle_joint` …). Declaring movable joints under vendor names would leave them
  permanently unpublished, so MoveIt's `CurrentStateMonitor` would never see a complete state and
  every Execute would abort — the same failure the `/clock` fix cured. Fixed geometry sidesteps it
  and matches the geometry-only decision below.
- **Geometry modelled OPEN (knuckle θ=0), `grasp_tcp` at CLOSED (θ=0.8, z=0.11181 m from the
  gripper base).** Open is the wider envelope so collision checking errs safe; the closed fingertip
  is where a grasped object actually sits, which is what a pose goal should mean. Cross-check: the
  same FK reproduces the 2F-85's published 85 mm stroke to 84.8 mm.
- **Mount is identity.** `tool0` is the ROS-Industrial tool frame (+Z out of the flange) and
  `robotiq_85_base_link` has +Z as its approach axis, so they already agree. See Known issues for
  the wrong turn taken here.
- `grasp_tcp`'s 0.11181 m is the finger-tip **link origin**, not the pad contact face, so
  `kinematics.py`'s `tcp_z_offset` should use it as a starting value and be trimmed once M4 can
  test a real grasp.

**M2 — The FollowJointTrajectory bridge.** ✅ **Done and verified live 2026-08-10**
(`trajectory_bridge.py`). Advertises `/scaled_joint_trajectory_controller/follow_joint_trajectory`
(the exact name `ur_moveit_config`'s `moveit_controllers.yaml` expects), consumes MoveIt's
`JointTrajectory`, and streams it to `/joint_command` at 60 Hz by sampling against elapsed
wall-clock time. Proof achieved: Execute in RViz moved the real Isaac Sim arm —
`Accepted trajectory with 49 points` → `Trajectory complete in 4.74s (worst joint error
0.0010 rad)` → `Execution completed: SUCCEEDED`.

> This only started working once `/clock` + `use_sim_time` were in place. Before that, MoveIt
> aborted at `Validating trajectory with allowed_start_tolerance 0.01` and the bridge was never
> contacted at all — see Known open issues.

**M3 — Planning scene collision objects.** Register the conveyor rails, belt surface, and table as
collision geometry so plans route around them. Source the measurements from
`isaac_conveyor_spawner.py`.

**M4 — Fixed-point pick-and-place.** `pick_orchestrator` node: approach a known, fixed pick pose →
close gripper → lift → transfer → place → open → home. Proves the full chain (planning, execution,
gripper) before adding motion.

**M5 — Dynamic intercept (the actual target).** A `conveyor_kinematics` node computes the cube's
live pose and picks an intercept point + arrival time; the orchestrator plans once to that
predicted pose, timed to finish on arrival. A single MoveIt plan takes 50–200 ms+ — too slow to
re-plan every frame against a moving target, so **predict-and-intercept** (compute where the item
*will* be, plan once, time execution to match) is the standard approach, not continuous
re-planning.

> Belt velocity is **not** available in `isaac_conveyor_spawner.py` (it only defines `SPAWN_POS`,
> `DESPAWN_Y`, and spawn timing) — the cubes move via the conveyor prim's PhysX surface velocity,
> set on the USD stage. M5 must either read that off the prim, or have the spawner record each
> cube's spawn wall-clock time and derive speed empirically from observed motion (also
> self-corrects for belt slip).

**M6 — Closed-loop tracking (stretch).** Only if M5's open-loop prediction proves insufficient:
feed `apriltag_pose.py`'s live pose stream into `moveit_servo` (already installed;
`ur_moveit_config` ships `config/ur_servo.yaml`) for continuous velocity-based tracking.

## Perception (AprilTag) plan

Deliberately lightweight — 36h11 AprilTags + OpenCV `solvePnP`, no GPU inference, no external
models. Kinematics chain: `base_link → … → tool0`.

1. Isaac Sim scene: fixed eye-to-hand camera over the workspace, a ~5 cm tagged cube
   (RigidBody + Collider), a bin as the place target, a **ROS2 Camera Helper** node in the Action
   Graph publishing `/rgb` + `/camera_info`. Read the camera extrinsic (`T_base_cam`) directly off
   the USD stage rather than measuring by hand.
2. Perception node: subscribe `/rgb` + `/camera_info`, run `apriltag_pose.py`, transform by the
   static extrinsic, publish the object pose in the robot base frame.
3. `pick_orchestrator`: on a detected tag, compute a top-down grasp pose, solve IK, execute
   approach → descend → close gripper → lift → move to bin → open → home.

## Order of work

```
M1 ──┬── M1b ──┐
     │         ├── M2 ── M3 ── M4 ── M5 ── M6
M0 ──┴─────────┘
```

M0, M1, M1b and M2 are built **and verified live** (2026-08-10) — the whole
plan→execute→gripper-aware chain works. Next up: **M3** (collision objects), with the perception
scene setup available in parallel since it only touches the Isaac Sim stage.

## Known open issues

- ~~**Sim-time vs. wall-clock.**~~ ✅ **RESOLVED 2026-08-10.** The Action Graph now publishes
  `/clock` (`ROS2 Publish Clock` ← `Isaac Read Simulation Time.simulationTime`, on the same
  `OnPlaybackTick`), and `moveit.launch.py` sets `use_sim_time: true` on all four nodes via a
  declared launch arg. **The severity in the old note was wrong and worth remembering:** the
  mismatch was a *soft* failure for planning (falls back to last-known joint values — verified:
  `/get_planning_scene` returned the correct live pose despite the warning) but a **hard blocker
  for execution**. `trajectory_execution_manager` calls `waitForCurrentState()` to validate the
  trajectory's start against `allowed_start_tolerance` (0.01 rad), that wait timed out, and every
  Execute aborted with `Failed to validate trajectory: couldn't receive full current joint state
  within 1s` — the bridge was never contacted. That is why M2 could never be verified.
- ~~**`rviz_env` FASTRTPS-stripping block is vestigial.**~~ ✅ **RESOLVED 2026-08-10** — deleted.
  The snap/GTK env-stripping block in the same file (works around VSCode's bundled GTK/libpthread
  conflicting with rviz2) is unrelated and still needed on Ubuntu.
- **Isaac Sim Stop/Play resets sim time to 0** (`Isaac Read Simulation Time.resetOnStop` defaults
  to `True`). ROS nodes that already saw t=1657 then see time jump backwards, which breaks them.
  **Operating rule: if you Stop and re-Play Isaac Sim, restart the ROS launch too.** Alternative if
  this becomes annoying: set `resetOnStop = False` so sim time increases monotonically across
  Stop/Play.
- ~~**`trajectory_bridge` samples with `time.monotonic()`.**~~ ✅ **RESOLVED 2026-08-10** — it now
  samples from `self.get_clock().now()` (sim time via `use_sim_time`), with the wall clock kept
  only as a stall guard so pausing Isaac Sim mid-trajectory aborts instead of hanging. **It bit
  exactly as predicted, one milestone later than the note guessed.** Raising RViz's velocity
  scaling from 0.1 to 0.5 pushed worst joint error from 0.0010 to **0.0095 rad** — against
  `allowed_start_tolerance` of 0.01 — so the *next* Execute intermittently aborted with
  `start point deviates from current robot state ... at joint 'elbow_joint'`. After the fix, the
  same speed tracks to **0.0000–0.0001 rad**. That also settled the open question: the cause was
  genuinely the clock mismatch, not Isaac Sim's articulation gains.
- **`allowed_start_tolerance` raised 0.01 → 0.05 rad** in `moveit.launch.py`. The 0.01 default
  (0.57°) is tight for a physics sim where the arm settles slightly under gravity between moves.
  Kept after the clock fix as belt-and-braces; it still catches a genuinely stale plan.
- **Execution speed lives in RViz, not in our config.** `ur_moveit_config`'s `joint_limits.yaml`
  defines *no* scaling factors, so the effective 0.1 comes from the RViz Motion Planning panel's
  own defaults (Velocity/Acceleration Scaling, Planning tab). MSA's generated `joint_limits.yaml`
  does set `0.1` but is not loaded. For scripted control later, use
  `setMaxVelocityScalingFactor()` / `setMaxAccelerationScalingFactor()`, which is how M4/M5 should
  run slow near the object and fast in transit.
- **USD link-frame *orientations* do not match the URDF's; positions do.** The mount rotation for
  M1b was first derived from `/World/ur5e/joints/robot_gripper_joint`'s `localRot0`, which rendered
  the gripper 90° off — that transform is expressed in *Isaac's* `wrist_3_link` frame, a different
  convention from the URDF's. The correct answer needed no measurement at all (identity, from the
  ROS-I `tool0` convention). **Rule: trust USD joint positions, derive orientations from ROS
  conventions.** `scripts/isaac_gripper_measure.py` has a related flaw — it compares
  `ComputeLocalToWorldTransform` across links that are flat siblings in the articulation, so its
  cross-link translations are meaningless. `scripts/isaac_joints_dump.py` supersedes it; the
  bounding boxes it prints are also world-axis-aligned, so they are only valid when the robot is
  axis-aligned.
- **MoveIt Setup Assistant needs `scripts/run_setup_assistant.sh`**, not a bare `ros2 launch` — the
  VSCode-snap GUI env kills it with the same `GLIBC_PRIVATE`/`libpthread` error that
  `moveit.launch.py` works around for rviz2. That workaround is **per-process**, so every new GUI
  binary hits it fresh.
- **`ur5e_gripper_moveit_config` must not be depended on** by `ur5e_isaac_control`: it already
  exec_depends on us (its config references our URDF), so the reverse is a cycle colcon refuses to
  order. Hence the SRDF is copied into `config/` rather than referenced.
- **PLAN.md previously claimed `trajectory_bridge`'s interpolation was "unit-tested".** There is no
  `test/` directory in the package — whatever testing happened was ad-hoc and never committed.
  Claim corrected 2026-08-10; a real test for `sample_trajectory()` is still worth writing.
- **Planning only succeeds for *small* interactive-marker drags** (plus the SRDF named poses `home`
  / `up`). Larger drags burn OMPL's full 5 s budget and return `FAILURE`. **Observed 2026-08-10,
  cause not yet confirmed.** Most likely benign: the UR5e's reach is ~850 mm, the RViz marker drags
  in the *view plane*, and it is easy to pull the goal outside the workspace without it looking far
  on screen — an unreachable pose gives OMPL no goal to grow toward, which produces exactly this
  signature. **Must be settled before M4**, since pick→lift→transfer→bin are large motions. Cheap
  discriminating test: set a large *joint-space* goal (e.g. `shoulder_pan` swung 180° from the
  Joints tab), which bypasses IK entirely — if that plans, the issue is purely reachability and
  there is nothing to fix. If it also fails, dig into planner/workspace bounds
  (`ValidateWorkspaceBounds` adapter, `default_workspace_bounds`).
- **Gripper as planning group vs. geometry-only:** decided — geometry-only, with gripper open/close
  driven directly over `/joint_command`, not through `move_group`. Two mimic-jointed fingers don't
  need motion planning.
- **8 GB VRAM** (RTX 4060 Laptop) is at Isaac Sim's stated minimum for a scene this size — watch
  for it once the conveyor + camera + collision scene is all loaded together.
