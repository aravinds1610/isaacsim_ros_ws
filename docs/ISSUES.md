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

- **The camera's view and the arm's reach barely overlap.** The camera is centred over a spot
  ~1.0 m from the base, so everything *reachable* lands near the edge of the frame — the cube
  currently sits at image `(1000, 81)` with ~53 px of clearance to the top edge. Fine for a static
  cube. **Must be fixed before M5**, when the cube moves along the belt and would leave frame:
  shift the camera so its centre sits over the reachable part of the pick zone, then re-run
  `isaac_camera_extrinsic.py`. Deliberately deferred until the pick zone is pinned down.

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
  for it once the conveyor + camera + collision scene is all loaded together.

## Settled decisions and config notes

- **Execution speed lives in RViz, not in our config.** `ur_moveit_config`'s `joint_limits.yaml`
  defines *no* scaling factors, so the effective 0.1 comes from the RViz Motion Planning panel's
  own defaults (Velocity/Acceleration Scaling, Planning tab). MSA's generated `joint_limits.yaml`
  does set `0.1` but is not loaded. For scripted control later, use
  `setMaxVelocityScalingFactor()` / `setMaxAccelerationScalingFactor()`, which is how M4/M5 should
  run slow near the object and fast in transit.

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
