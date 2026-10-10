# UR5e AprilTag Sorting Cell — Plan

**Purpose:** learn Isaac Sim + ROS 2 Jazzy by building an AprilTag-driven pick-and-place.
**End state:** cubes spawn on a running conveyor at an interval; the camera reads each cube's tag
as **Good** or **Bad**; the UR5e picks Good cubes *off the moving belt* into the collection bin
(`RecollectionBin`); Bad cubes ride off the belt end into the waste bin (`Sorting_Bin_1`).

**Stack:** Isaac Sim 6.0.1 (`~/isaacsim`, native) · ROS 2 Jazzy (`/opt/ros/jazzy`, native) ·
Isaac ROS 4.6 container for planning (`isaac-ros activate`) · UR5e + Robotiq 2F-85 ·
workspace `~/isaacsim_ros_ws`. History, scene numbers and M0–M4 lessons:
[ARCHIVE_PLAN_M0-M4.md](ARCHIVE_PLAN_M0-M4.md). Traps and decisions: [ISSUES.md](ISSUES.md).

## How I'd like you to work with me

- **Concept before commands.** Explain what a step does and why, before running it.
- **One step at a time.** Stop and check I've understood before moving on.
- **Be explicit about verified vs. assumed.** "I haven't confirmed this" beats a confident guess.
- **Correct me plainly** if I state something wrong.

I have hands-on Isaac Sim experience; started MoveIt/ROS 2 at zero.

## Architecture

```
Isaac Sim (host)              ROS 2 host                          Isaac ROS container
----------------              ----------                          -------------------
camera ─▶ /rgb ─────────────▶ cube_pose_publisher ─/cube_pose─┐
                                                              ▼
                              pick_orchestrator ─/compute_ik, /move_action─▶ move_group + cuMotion
UR5e ◀─ /isaac_joint_commands ── (gripper only)                                  │
     ◀─ /isaac_joint_commands ─────────────────────────────── joint_trajectory_controller
     ─▶ /isaac_joint_states ──────────────────────────────▶   (ros2_control, topic_based)
```

The container runs with `--network host`, so all three share one ROS graph (verified 2026-10-10).
The container side is NVIDIA's `ur_isaac_sim.launch.py` with two fixes for defects in their
release (ISSUES.md → Settled). Its files are versioned in `isaac_ros/`, laid out like
`~/workspaces/isaac_ros-dev/`; after editing, sync with
`cp -r ~/isaacsim_ros_ws/isaac_ros/. ~/workspaces/isaac_ros-dev/` (and `isaac-ros activate
--build-local` if the Dockerfile changed).

## Running a pick

Isaac: open `scenes/UR5e_Sim_V1.usd` (the repo copy), press Play, cube in view.

```bash
# 1. container (host terminal)
isaac-ros activate
ros2 launch /workspaces/isaac_ros-dev/launch/ur_isaac_sim.launch.py ur_type:=ur5e
# 2. host
ros2 launch ur5e_isaac_control pick_cumotion.launch.py
# 3. host
ros2 service call /pick_orchestrator/pick_once std_srvs/srv/Trigger
```

**Stop/Play in Isaac resets sim time → restart 1 and 2.** To reset the cube, drag it while playing.
Record a run for analysis: `ros2 bag record -o ~/bags/<name> /clock /isaac_joint_commands
/isaac_joint_states /cube_pose`.

## Done

| | Verified |
|---|---|
| M0–M2 · gripper wiring, MoveIt bring-up, `trajectory_bridge` on sim time | 2026-08-10 |
| Perception · `/cube_pose` from AprilTag, ~0.3 mm error | 2026-08-11, re-checked 09-27 |
| M4 · fixed-point pick → bin with Pilz | 2026-09-27 |
| C1 setup · cuMotion plans, ros2_control executes, full pick → bin (`~/bags/cumotion_pick2`, `_pick3`): no elbow flip, worst tracking 0.046 rad, fingertips within ~1 mm of target at grasp, cuMotion uses ~106 MB GPU | 2026-10-10 |

## Milestones

Each one ends with a recorded check (`ros2 bag` or a log) that passes or fails.

**C1 (last item) — Straight vertical moves.** cuMotion's "LIN" moves curve ~10 mm sideways, which
likely caused one missed grasp. Use `/compute_cartesian_path` + `/execute_trajectory` for descend,
lift, lower and retreat (details in ISSUES.md).
- **Pass:** recorded descent deviates ≲ 1 mm; 3 consecutive full picks succeed.

**C2 — Bins and static collision scene.**
1. Swap the bins in the stage: `RecollectionBin` onto the block beside the robot (where
   `place_xyz` already points), `Sorting_Bin_1` under the belt's far end.
2. A `.scene` file with 4–5 boxes measured from the stage: robot pedestal, bin block + bin walls,
   conveyor **belt surface only** (a full conveyor box makes every grasp collide — archive, M3).
   Loaded into the container's planning-scene server.
- **Pass:** boxes visible in RViz; a goal inside the pedestal is rejected; picks still succeed.

**M5 — Pick from the moving belt.** Belt on at a fixed speed. Predict where the cube will be
(pose + belt velocity from successive `/cube_pose`), plan once to intercept above it, then follow
the belt while descending, closing and lifting.
- **Open design question:** how to follow the belt during the grasp — `moveit_servo`, a cuMotion
  controller, or timed Cartesian moves. Decide at the start of M5.
- **Pass:** 5/5 picks at the chosen belt speed, cube ends in the collection bin.

**M6 — Spawn and sort.** Spawner drops cubes at an interval, each tagged Good or Bad by **tag ID**
(e.g. id 0 = Good, id 1 = Bad). The perception node publishes the ID with the pose; the
orchestrator picks only Good. Bad cubes ride off the belt end into `Sorting_Bin_1`.
- **Pass:** over 20 spawned cubes, every Good one in `RecollectionBin`, every Bad one in
  `Sorting_Bin_1`, none dropped.

## Known risks

- **GPU memory (8 GB).** Isaac ~5.2 GB, cuMotion ~0.1 GB, total 6.2 GB; GPU at 100%, sim at 0.41×
  real time. Measure every new GPU consumer before keeping it. No nvblox / depth camera planned
  (no moving obstacles expected).
- **Tag detection at ~4 px/cell** is at the limit; a moving cube may need a closer camera or a
  bigger tag.
- **NVIDIA's bare-arm model has no gripper**, so cuMotion doesn't collision-check the gripper. The
  orchestrator aims `tool0` with `tcp_offset_z = 0.11181` to put the fingertips on target.
- **The old Pilz path** (`pick_demo.launch.py`, `trajectory_bridge`) only works if the stage's
  joint topics are renamed back to `/joint_states` / `/joint_command`.
