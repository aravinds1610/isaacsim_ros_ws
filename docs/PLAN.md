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
> discovery is expected to just work. If `ros2 topic list` or MoveIt's plan/execute behave oddly, it
> is a *new* problem, not a recurrence of the DDS bug — check `~/.bashrc` has no leftover
> `FASTRTPS_DEFAULT_PROFILES_FILE` and that `launch/moveit.launch.py`'s `rviz_env` var-stripping
> (a WSL2-era leftover, now a harmless no-op) isn't masking something before assuming otherwise.

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
| `src/ur5e_isaac_control/urdf/ur5e.urdf` | Robot description (robot name `ur5e`) |
| `scripts/isaac_conveyor_spawner.py` | Spawns cubes on the conveyor in Isaac Sim |
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
| MoveIt bring-up (`moveit.launch.py`) — **M1** | ✅ built, planning previously verified (under the old WSL2 setup — re-verify natively) |
| `trajectory_bridge` (FollowJointTrajectory → `/joint_command`) — **M2** | ✅ built, interpolation unit-tested; **live execution never verified end-to-end** |
| URDF gripper geometry + `grasp_tcp` frame — **M1b** | ⬜ not started |
| Isaac Sim scene: camera + tagged cube + bin — perception scene setup | ⬜ not started |
| Perception node (subscribe `/rgb`, publish object pose in base frame) | ⬜ not started |
| Planning-scene collision objects (conveyor, table) — **M3** | ⬜ not started |
| Fixed-point pick-and-place (`pick_orchestrator`) — **M4** | ⬜ not started |
| Dynamic intercept (`conveyor_kinematics`, predict-and-intercept) — **M5** | ⬜ not started |
| Closed-loop tracking via `moveit_servo` (stretch) — **M6** | ⬜ not started |

**Immediate next step:** re-run the live MoveIt test (see EXPLAINED.md §7) now that Isaac Sim and
ROS2 share one OS — Plan *and* Execute should both work without any DDS workaround. This
re-verifies M1+M2 under the current environment before starting M1b/M3.

## Milestones

**M0 — Gripper ROS2 wiring.** ✅ Done. The 2F-85 is controllable via `/joint_command`'s
`finger_joint`.

**M1 — MoveIt bring-up.** ✅ Built. Custom launch file (`moveit.launch.py`) runs
`robot_state_publisher` with our URDF, parameterizing `move_group` from `ur_moveit_config`'s SRDF/
kinematics/OMPL/joint-limits YAML — `ur_moveit_config`'s own launch file can't be used as-is (see
EXPLAINED.md §3). Proof target: RViz Motion Planning panel, drag the marker, Plan succeeds, ghost
arm animates.

**M1b — Extend the URDF with gripper geometry + `grasp_tcp` frame.** The official SRDF has no
gripper group — MoveIt currently plans to `tool0` (the bare flange), not the fingertips, and can't
see the gripper for collision checking. Add the 2F-85 as fixed collision geometry on `tool0` plus a
`grasp_tcp` fixed frame at the finger tips. Also lets `kinematics.py`'s `tcp_z_offset` argument
finally get a real measured value.

**M2 — The FollowJointTrajectory bridge.** ✅ Built (`trajectory_bridge.py`). Advertises
`/scaled_joint_trajectory_controller/follow_joint_trajectory` (the exact name
`ur_moveit_config`'s `moveit_controllers.yaml` expects), consumes MoveIt's `JointTrajectory`, and
streams it to `/joint_command` at 60 Hz by sampling against elapsed wall-clock time. Proof target:
Execute in RViz moves the real Isaac Sim arm — **this is the one thing not yet confirmed.**

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

M0, M1, M2 are already built. Next up: re-verify M1+M2 live on native Ubuntu, then M1b and the
perception scene setup, then M3 onward.

## Known open issues

- **Sim-time vs. wall-clock.** Isaac Sim stamps `/joint_states` with simulation time (seconds
  since Play); `move_group`'s `current_state_monitor` expects wall-clock and can log "didn't
  receive robot state within 1.0s" against a huge apparent gap. Soft failure — planning still works
  off the last known joint values — but not yet fixed. Likely fix: publish `/clock` from the
  Action Graph and set `use_sim_time: true` on all nodes. Not yet attempted.
- **`moveit.launch.py`'s `rviz_env` FASTRTPS-stripping block is vestigial** now that no unicast DDS
  profile is set anywhere — harmless no-op, but worth deleting during the next MoveIt-launch-file
  edit so it doesn't read as active behavior. The snap/GTK env-stripping block in the same file
  (works around VSCode's bundled GTK/libpthread conflicting with rviz2) is unrelated and still
  needed on Ubuntu.
- **Gripper as planning group vs. geometry-only:** decided — geometry-only, with gripper open/close
  driven directly over `/joint_command`, not through `move_group`. Two mimic-jointed fingers don't
  need motion planning.
- **8 GB VRAM** (RTX 4060 Laptop) is at Isaac Sim's stated minimum for a scene this size — watch
  for it once the conveyor + camera + collision scene is all loaded together.
