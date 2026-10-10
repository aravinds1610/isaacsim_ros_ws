# Known Issues & Gotchas

Companion to [PLAN.md](PLAN.md). Everything here was hit for real on this setup — active traps
first, then settled decisions, then resolved issues kept because the *reason* is worth remembering.

## Active

- **⚠️ Dragging a prim that is welded by a `FixedJoint` does not move it.** A `FixedJoint` stores
  the *relative pose between two bodies*, baked in when it was authored. Dragging the child in the
  viewport changes its stage transform but leaves that stored pose untouched — so on **Stop** you
  see where you dragged it, and on **Play** PhysX restores the old relative pose and the body
  appears to jump or hover. Hit on 2026-08-11 when moving the robot closer to the belt. Three ways
  out: move parent and child **together** (relative pose unchanged, nothing to fix — the easiest);
  run `scripts/isaac_rebake_fixed_joint.py` to rewrite the joint's local frames to the current
  placement; or delete and recreate the joint in the GUI.

- **⚠️ Reach is a real constraint on scene layout, and it is easy to violate.** The UR5e reaches
  **0.850 m** (`kinematics.py` measured 0.852 m at zero pose); `grasp_tcp` adds 0.112 m but only at
  a fully-extended singularity. As first built, the cube sat **1.045 m** from the base — flatly
  unreachable, and the failure at M4 would have looked exactly like the "large drags fail"
  symptom. Confirmed visually by commanding the arm straight out at the cube's azimuth
  (`joint_command_publisher --positions "1.5339,0,0,0,0,0"`) and watching it stop ~20 cm short.
  Fixed by moving the robot **and** cube; now 0.745 m. **Rule: after moving anything, re-check the
  base→cube distance before debugging the planner.**

- **⚠️ USD link-frame *orientations* do not match the URDF's; positions do.** The mount rotation for
  M1b was first derived from `/World/ur5e/joints/robot_gripper_joint`'s `localRot0`, which rendered
  the gripper 90° off — that transform is expressed in *Isaac's* `wrist_3_link` frame, a different
  convention from the URDF's. The correct answer needed no measurement at all (identity, from the
  ROS-I `tool0` convention). **Rule: trust USD joint positions, derive orientations from ROS
  conventions.**

  Consequence still open: **USD `base_link`'s yaw relative to the URDF's `base_link` is
  unconfirmed.** The camera extrinsic and the "ground truth" it was validated against both come
  from USD's `base_link`, so they share that assumption — a shared yaw error would leave both wrong
  together and the sub-millimetre check would still pass. USD's `base_link` has identity
  orientation in world and its +Z (shoulder-pan axis) is correctly vertical. **Settle before M4:**
  compare `kinematics.py`'s FK for `tool0` in `base_link` against `tool0`'s USD world pose minus
  the base origin, or render `/cube_marker` in RViz beside the robot model and check it lands where
  the cube physically is.

- **The camera's view and the arm's reach barely overlapped — partly addressed 2026-09-27.** As
  built, the camera was centred ~1.0 m from the base, so everything *reachable* landed near the
  frame edge (cube at image `(1000, 81)`, ~53 px from the top). The camera has since been raised
  0.72 m and switched to 1080p / focal 40, covering ~1.64 × 0.92 m; the cube at base
  `(+0.004, +0.600)` shows in the upper-right quadrant at `(1198, 225)`. **Still to do before M5:**
  pin down the pick zone and centre the camera over its reachable part, then re-run
  `isaac_camera_extrinsic.py`.

- **⚠️ Zooming in the viewport while looking through `/World/Camera` moves the camera prim.** It is
  not a lens zoom — it dollies the actual camera, silently invalidating the extrinsic. Hit
  2026-09-27: the camera moved 0.72 m up and 0.20 m sideways, and every `/cube_pose` came out
  **28 cm** wrong while detection itself was perfect (re-projecting the tag with the camera's *new*
  pose landed within 0.6 mm of the USD cube). **Rule: change the view with
  `isaac_camera_setup.py` (focal length), never by scrolling through the camera; after any camera
  change, compare `/World/Camera`'s Translate against PLAN.md's scene numbers and re-run
  `isaac_camera_extrinsic.py` if it moved.** The durable fix is to have Isaac publish the camera's
  pose to TF at run time (`ROS2PublishTransformTree`) instead of hard-coding it — not yet done.

- **⚠️ An orthographic camera silently destroys AprilTag pose estimation.** `/World/Camera` was
  authored `projection = orthographic`. In orthographic there is no perspective, so apparent size
  carries **no depth information** and `solvePnP` cannot recover a pose — it returns confident,
  meaningless numbers. Two things make this vicious: (a) **Isaac publishes a perfectly well-formed
  pinhole `camera_info` anyway** (it applies `fx = W · focal / aperture` regardless), so nothing
  downstream can tell; (b) `focalLength` is *ignored* in orthographic, so three rounds of changing
  it — 120, 80, 200 — did nothing at all while the attribute readback dutifully confirmed each new
  value. The tell was that the published `fx` (3680) was **exactly 3×** what the image behaved like
  (~1225), and that the orthographic aperture (27.8254, in USD's tenths-of-a-world-unit) works out
  to a 2.78 m view — which is what the image actually showed. `isaac_camera_setup.py` now forces
  perspective and `isaac_camera_check.py` refuses to run without it. **Rule: measure the image,
  don't trust `camera_info`.** Verified 2026-08-11 — a projection change applies to the viewport
  *instantly*, no Stop/Play needed, which is what finally proved the diagnosis.

- **Detection needs the tag ≈40 px across, and it fails silently below that.** A 36h11 tag is 8
  cells wide, so decoding needs several pixels per cell. Measured 2026-08-11: on a *clean
  synthetic* image the detector works down to **18.5 px** (2.3 px/cell), but the real Isaac render
  **failed at 16 px** — render noise, anti-aliasing and shadows eat the margin. Working target is
  40 px; the current setup gives 56.8. Failure mode is zero detections, which looks exactly like a
  broken pipeline. `isaac_camera_check.py` predicts this before anything is built.

  **Update 2026-09-27:** after the camera change the black square is **~32 px (4 px/cell)** and the
  default OpenCV detector rejected every candidate — on a sharp, clearly readable tag. Offline
  tests on a saved frame: upscaling 2× did not help, wider threshold windows did not help;
  **`perspectiveRemovePixelPerCell` 4 → 8 with `perspectiveRemoveIgnoredMarginPerCell` 0.13 → 0.2
  did** (now set in `apriltag_pose.py`). These only change how bits are *read*; corners still come
  from the contour, so pose accuracy is unaffected. Detection is still **intermittent** at this
  size — fine for a latched static pick, not for M5. Focal 50 at 1080p (~5 px/cell) is the easy
  margin. Note `isaac_camera_setup.py`'s pixel prediction includes the tag's white border, so it
  reads ~30% high against the black square that actually matters.

- **`isaac_conveyor_spawner.py` has two stale/wrong lines.** `TAG_TEXTURE` still points at
  `\\wsl.localhost\Ubuntu\...\isaac_sim_ros\apriltags\...` — a Windows UNC path from the retired
  WSL2 setup, to a directory that no longer exists; on native Ubuntu it must be
  `/home/aravindsuresh16/isaacsim_ros_ws/apriltags/apriltag_36h11_id0.png`. And the comment on
  `TAG_SIZE = 0.045` calls it the "black-square edge" — it is not, it is the **quad** edge, because
  `_add_tag_plane` maps UV 0→1 over the whole PNG including the quiet zone. The black square is
  `0.045 × 0.769 = 0.0346`. Passing 0.045 to `AprilTagDetector` would report every distance 30% too
  far. Not yet fixed — the static cube uses `isaac_tag_decal.py` instead, which gets this right.

- **Isaac Sim Stop/Play resets sim time to 0** (`Isaac Read Simulation Time.resetOnStop` defaults
  to `True`). ROS nodes that already saw t=1657 then see time jump backwards, which breaks them.
  **Operating rule: if you Stop and re-Play Isaac Sim, restart the ROS launch too.** Alternative if
  this becomes annoying: set `resetOnStop = False` so sim time increases monotonically.
  Concrete casualty, 2026-09-27: `pick_orchestrator` kept a pose from before the reset, whose age
  came out **negative** and so passed `age <= 5 s` — it re-picked the cube's old position after the
  cube had been moved. Fixed (`0 <= age`), but any other age check is exposed the same way.

- **`scripts/isaac_gripper_measure.py` gives unreliable cross-link transforms.** It compares
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

- **`trajectory_bridge`'s interpolation has never been unit-tested.** PLAN.md previously claimed it
  was; there is no `test/` directory in the package, so whatever testing happened was ad-hoc and
  never committed. Claim corrected 2026-08-10; a real test for `sample_trajectory()` is still worth
  writing.

- **8 GB VRAM** (RTX 4060 Laptop) is at Isaac Sim's stated minimum for a scene this size — watch
  for it once the conveyor + camera + collision scene is all loaded together. Measured 2026-09-27
  with the 1080p camera, viewport and RViz running: **6.4 / 8.2 GB, GPU at 100%, sim at ~0.4× real
  time.** Everything runs on sim time so it still works, just slowly.

- **`wrist_1` lagged 1.67 rad behind its command on one move, cause unknown (2026-09-27).** On the
  PTP from straight-up to the ready pose, `wrist_1`'s tracking error grew to 1.67 rad while
  pan/lift/elbow stayed within ~0.1 rad; the arm then reached the target during the settle window.
  Earlier, a slow (0.1 scaling) approach showed `shoulder_pan` falling behind at a steady
  ~0.07 rad/s from 12 s in, then also reaching the target. Ruled out: a joint velocity cap
  (`Maximum Joint Velocity` = inf) and `wrist_2`'s drive on its own (clean in mid-air slider
  tests). Drive values as authored: stiffness 9400 (pan) / 3940 (wrists) **per degree**, max force
  150 / 28 N·m, damping 0.378 (pan), **1.579 (wrist_1) vs 0.061 (wrist_2)** — the 26× wrist
  damping mismatch on otherwise identical joints is unexplained. Masked by `goal_time_sec`;
  **must be understood before M5**, where arriving late is failing. Diagnostic tool: the per-joint
  error recorder that subscribes to the bridge's action feedback
  (`/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/feedback`).
  **2026-10-10:** reproduced on the Pilz/bridge path (`~/bags/jitter_run1..3`): `wrist_1` is pushed
  the *wrong way* by ~1 rad during the two big PTPs while pan/lift/elbow move, then snaps back at
  ~3 rad/s — the visible "jitter". Ruled out: torque cap (Max Force 28 → 150, now saved in the
  stage, helped only partly), collision (colliders watched, nothing touched), passive inertia (wrist
  world pitch swings rather than holding). **Not seen on the cuMotion path** (`cumotion_pick2`:
  `wrist_1` max error 0.031 rad) — cause still unknown; which change cured it (Max Force, JTC spline
  interpolation, or cuMotion's paths) was not isolated.

- **cuMotion's "LIN" moves are not straight — likely cause of occasional missed grasps (2026-10-10).**
  cuMotion plans in joint space and has no Cartesian-line mode, so `pick_cumotion.launch.py` maps
  `LIN` to `cuMotion`. Measured on a *successful* pick (`~/bags/cumotion_pick3`): the fingertips
  wander up to 10.6 mm sideways during the last 13 cm of descent (9.8 mm at cube level), against
  ~17.5 mm per-finger clearance (85 mm stroke, 50 mm cube; 71 mm across a corner). One unrecorded
  run pushed and rotated the cube instead of grasping it. **Fix, not yet built:** for descend /
  lift / lower / retreat, call `/compute_cartesian_path` and run the result with
  `/execute_trajectory` (both verified available in NVIDIA's `move_group`); fail the step if the
  path is under 100% complete. Check: descent deviation ≲ 1 mm in a recorded run.

- **Bins are the wrong way round for the robot (2026-10-10).** Roles: `RecollectionBin` = collection
  (Good), `Sorting_Bin_1` = waste (Bad). `place_xyz` targets `Sorting_Bin_1` (on the block beside the
  robot); `RecollectionBin`'s origin is ~2.7 m from the base, on the floor — out of reach. Swap their
  positions in the stage (or rename) before M6.

- **`ros2 bag record` silently skips hidden topics even when named explicitly** — e.g. an action's
  `…/_action/feedback`. Record `/isaac_joint_commands` + `/isaac_joint_states` instead; they carry
  the same target-vs-actual information.

- **`timeout N ros2 topic hz|echo … | grep` prints nothing even when the topic is live.** Python
  block-buffers stdout into a pipe, and `timeout` kills it before the buffer is flushed — so an
  empty result is an artefact, not evidence. Cost a wrong "no images arriving" diagnosis on
  2026-09-27. Use `PYTHONUNBUFFERED=1`, `--once` (exits cleanly), or redirect to a file.

## Settled decisions and config notes

- **cuMotion stack = NVIDIA's `ur_isaac_sim.launch.py` with two fixes (2026-10-10).** Both are
  defects in NVIDIA's Isaac ROS 4.6 release, not in our setup:
  1. **Prebuilt `ros-jazzy-topic-based-ros2-control` (99.99.1) segfaults** in
     `HardwareComponent::initialize()` → `get_lifecycle_id()` against Jazzy's ros2_control 4.48.1
     (gdb-confirmed): it was compiled against an older `hardware_interface` (ABI mismatch; its
     package declares no version bound, so apt can't tell). `isaac_ros/docker/Dockerfile.cumotion`
     recompiles upstream at commit `aa9f9ef` over the apt copy.
  2. **The example hard-codes `ur10e.xrdf`** whatever `ur_type` is passed. UR10e collision spheres on
     a UR5e make most start states "collide" → `INVALID_INITIAL_CSPACE_POSITION` (17/18 plans
     failed). Our copy (`isaac_ros/launch/ur_isaac_sim.launch.py`) uses `{ur_type}.xrdf`.
- **Pose goals to cuMotion can flip the elbow; send joint goals instead (2026-10-10).** cuMotion
  solves pose goals with its own many-seed IK and picks any branch: "above bin" went elbow
  +1.68 → −0.34 rad and swung `shoulder_lift` 118°. `ik_then_joint_goal` makes the orchestrator get
  IK from `/compute_ik` (seeded from the current state) and send a joint goal — what RViz does.
  Caveat: KDL falls back to random restarts, so seeded ≠ guaranteed nearest — seeded from *home*,
  the bin pose came back on a far branch; from the real (post-lift) state it was consistent 5/5.
- **Isaac joint topics are now `/isaac_joint_states` / `/isaac_joint_commands`** (NVIDIA's names, set
  in the stage's Action Graph 2026-10-10). The old Pilz path (`pick_demo.launch.py`,
  `trajectory_bridge`) expects `/joint_states` / `/joint_command` and will not work unless they are
  renamed back.

- **Execution speed is set per request, not in config.** `ur_moveit_config`'s `joint_limits.yaml`
  defines *no* scaling factors, so RViz's 0.1 comes from its Motion Planning panel's own defaults
  (Planning tab). MSA's generated `joint_limits.yaml` sets `0.1` but is not loaded. The
  orchestrator sets `max_velocity/acceleration_scaling_factor` on every `MoveGroup` request from
  its `velocity_scaling` / `acceleration_scaling` parameters (**0.3** since 2026-09-27).

- **Pilz for scripted motion, OMPL left as the default pipeline (2026-09-27).** OMPL paths are
  random; Pilz PTP/LIN are deterministic. The orchestrator sets `pipeline_id` + `planner_id` per
  request, so RViz's dropdown does not affect the pick. For hand-planning in RViz, pick
  `pilz_industrial_motion_planner` / `PTP` in the Context tab, or flip `default_planning_pipeline`
  in `moveit.launch.py`. Pilz does **not** avoid obstacles — it validates a path and refuses a
  colliding one — and PTP only makes the *path* deterministic: the goal joints still come from IK.

- **Never start a pose move from the straight-up pose.** `[0, -π/2, 0, 0, 0, 0]` (SRDF `home`/`up`)
  is a singularity; IK seeded there picks an arbitrary branch, and a PTP to it swept the upper arm
  through the base (2026-09-27). The orchestrator's `HOME_JOINTS` is an elbow-up, tool-down ready
  pose instead. Same root cause as RViz marker drags sometimes swinging "all the way round": RViz
  seeds IK from the marker's previous solution, not from the robot.

- **`allowed_start_tolerance` raised 0.01 → 0.05 rad** in `moveit.launch.py`. The 0.01 default
  (0.57°) is tight for a physics sim where the arm settles slightly under gravity between moves.
  Kept after the clock fix as belt-and-braces; it still catches a genuinely stale plan.

- **Gripper: geometry-only, not a planning group.** Open/close is driven directly over
  `/joint_command`, not through `move_group`. Two mimic-jointed fingers don't need motion planning.

- **Large interactive-marker drags failing to plan — largely settled 2026-08-11.** The
  discriminating test: a multi-joint goal set from RViz's **Joints** tab (which bypasses IK
  entirely, so only OMPL is exercised) planned in **17 ms and 8 ms** — RRTConnect, nowhere near the
  5 s budget, `CheckStartStateBounds` and `CheckStartStateCollision` both clean. So the planner and
  workspace bounds are fine and the earlier failures were unreachable Cartesian poses, as
  suspected. **Honest caveat:** the logs record that a plan succeeded but not how *far* the goal
  was, and the follow-up measurement of the trajectory's joint-space span was not run. If large
  motions fail during M4, reopen this rather than assuming it is closed. Practical habit that came
  out of it: when planning fails, first re-test with a named joint-space goal (`home`) to tell
  "goal was bad" apart from "the stack is broken".

## Resolved

- ✅ **"Goal tolerance exceeded" after long moves was lag, not a collision (2026-09-27).** The first
  M4 run aborted with the arm ~0.52 rad off its goal, and the per-joint trace (one joint tracking,
  then falling behind linearly) was first read as the arm jamming against the conveyor. Wrong:
  forward kinematics on `/joint_states` afterwards put the TCP **3 mm / 2°** from the goal — the arm
  had arrived, late. A blocked arm stops *short*; a lagging one gets there. Fixed with the bridge's
  `goal_time_sec` settle window; the lag itself is an open issue above. **Rule: before calling it
  a collision, check where the arm ended up.**

- ✅ **Wrist spun ~25 turns and knocked the cube away at the grasp (2026-09-27).** `grasp_z_offset`
  0.0 put the fingertip link origin at the cube centre, 25 mm above the belt; the finger pads reach
  ~30–40 mm past that origin, so they drove into the belt and the contact fight with the stiff
  joint drives blew up the physics (`wrist_2` at 156.6 rad, `finger_joint` at −1.19). Mid-air
  tests of `wrist_2` and the gripper were clean, isolating contact as the trigger. Fixed with
  `grasp_z_offset` +0.02; M4 then completed. **Symptom to recognise:** joint values far outside
  anything commanded = PhysX blow-up, look for an unintended contact.

- ✅ **`kinematics.py` could not run — `ikpy` was missing (2026-09-04).** Installed user-local with
  `pip install --user --no-deps --break-system-packages ikpy` (4.1.0). **`--no-deps` is the
  important part:** ikpy wants numpy/scipy/sympy, all three are already present from apt, and
  letting pip install its own copies into `~/.local` would shadow the apt ones for *every*
  system-python process — ROS nodes included. With `--no-deps` the only thing in
  `~/.local/lib/python3.12/site-packages` is ikpy. Both documented figures reproduce: FK at the
  zero pose puts `tool0` at 0.8521 m from the base (the 0.852 m reach number above), and a
  200-pose IK round-trip returns 0.0000 mm / 0.0000°. Note ikpy has **no rosdep key**, so it is
  deliberately not declared in `package.xml` — see the comment there. It also emits harmless
  `UserWarning`s about fixed links being in the active mask.

- ✅ **Sim-time vs. wall-clock (2026-08-10).** The Action Graph now publishes `/clock`
  (`ROS2 Publish Clock` ← `Isaac Read Simulation Time.simulationTime`, on the same
  `OnPlaybackTick`), and `moveit.launch.py` sets `use_sim_time: true` on all four nodes via a
  declared launch arg. **The severity in the old note was wrong and worth remembering:** the
  mismatch was a *soft* failure for planning (falls back to last-known joint values — verified:
  `/get_planning_scene` returned the correct live pose despite the warning) but a **hard blocker
  for execution**. `trajectory_execution_manager` calls `waitForCurrentState()` to validate the
  trajectory's start against `allowed_start_tolerance`, that wait timed out, and every Execute
  aborted with `Failed to validate trajectory: couldn't receive full current joint state within
  1s` — the bridge was never contacted. That is why M2 could never be verified.

- ✅ **`trajectory_bridge` sampled with `time.monotonic()` (2026-08-10).** Now samples from
  `self.get_clock().now()` (sim time via `use_sim_time`), with the wall clock kept only as a stall
  guard so pausing Isaac Sim mid-trajectory aborts instead of hanging. **It bit exactly as
  predicted, one milestone later than the note guessed.** Raising RViz's velocity scaling from 0.1
  to 0.5 pushed worst joint error from 0.0010 to **0.0095 rad** — against `allowed_start_tolerance`
  of 0.01 — so the *next* Execute intermittently aborted with `start point deviates from current
  robot state ... at joint 'elbow_joint'`. After the fix, the same speed tracks to
  **0.0000–0.0001 rad**. That also settled the open question: the cause was genuinely the clock
  mismatch, not Isaac Sim's articulation gains.

- ✅ **`apriltag_pose.py` required OpenCV 4.7+ (2026-08-11).** It used `cv2.aruco.ArucoDetector`,
  added in 4.7; Ubuntu 24.04 / ROS 2 Jazzy ship **4.6.0**, where that class does not exist and the
  4.7-style `DetectorParameters()` **segfaults**. Now branches on `_HAS_ARUCO_DETECTOR` and works
  on both. Upgrading OpenCV was considered and rejected: the system Python is PEP-668 managed,
  `cv_bridge` links `libopencv_core.so.406`, and a pip `cv2` 4.12 would put two OpenCVs in one
  process for no capability gain (we only need `detectMarkers` + `solvePnP`, both present in 4.6).
  This also exposed a false claim: `apriltag_pose.py` had been listed "built & validated" when the
  validation covered only the pose *maths* — the detection path had never run on this machine.

- ✅ **WSL2 DDS workaround retired (2026-08-10).** The `rviz_env` FASTRTPS-stripping block in
  `moveit.launch.py` is deleted. The snap/GTK env-stripping block in the same file (works around
  VSCode's bundled GTK/libpthread conflicting with rviz2) is unrelated and still needed.
