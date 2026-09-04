# UR5e Conveyor Pick — Explained From Zero

A companion to [PLAN.md](PLAN.md). The plan says *what* we're building and in what order; this file
explains *why each piece exists* and what the concepts mean, assuming no prior MoveIt or
motion-planning knowledge. Written as the project progresses, so it tracks what has actually been
built and verified — not generic tutorial material.

---

## 1. The core concept: MoveIt does not move robots

This sounds absurd, but it is the single idea the whole architecture follows from.

**MoveIt is a planner, not a driver.** Think of it as a GPS: it computes a route, but it never
touches the steering wheel.

You give MoveIt a goal — "put the gripper *here*" — and it computes:

- **Inverse kinematics (IK)**: which 6 joint angles place the gripper at that pose. (Usually
  several answers exist; the solver picks one.)
- **A collision-free path**: a sequence of intermediate joint positions that gets from the current
  pose to the goal without driving the arm through the conveyor, the table, or itself.
- **Timing**: how fast to move through each point without exceeding the joints' velocity and
  acceleration limits.

The output is a **trajectory** — a list of waypoints, each of the form *"at t = 0.7 s, the six
joints should be at these angles."*

MoveIt then hands that list to something else to actually execute it.

Two refinements to the "backend calculator" shorthand, both of which matter later:

- `move_group` is not purely passive. It also **dispatches and monitors** execution — it sends the
  trajectory out and waits to hear whether it succeeded. This is why our bridge doesn't simply
  receive a trajectory and go quiet — it must report progress and a final success/failure result
  back, so MoveIt can decide whether to continue or abort a sequence.
- `move_group` also owns the **world model** (the planning scene, Section 2) — a calculator *with
  memory of the environment*, which is what makes milestone M3 meaningful.

### The gap in our setup

MoveIt delivers trajectories over a ROS2 **action** named `FollowJointTrajectory`. On a real UR5e,
the `ros2_control` framework provides that action. Isaac Sim does not.

What Isaac Sim gives us instead is a plain **topic**, `/joint_command`: publish a
`sensor_msgs/JointState` and the Action Graph's Articulation Controller applies those angles.

Those are fundamentally different interfaces:

| | MoveIt's side | Isaac Sim's side |
|---|---|---|
| Mechanism | Action (`FollowJointTrajectory`) | Topic (`/joint_command`) |
| Payload | A whole timestamped plan | One pose, "go here now" |
| Feedback | Progress + success/failure result | None |

So there is a hole in the middle:

```
MoveIt ──[ FollowJointTrajectory action ]──> ??? ──[ /joint_command topic ]──> Isaac Sim
                                             ^
                                    this is what's missing
```

Filling that hole is the job of `trajectory_bridge.py` (Section 5) — the only genuinely new
*logic* in this project; everything else is configuration or wiring.

---

## 2. Vocabulary you need

**URDF** — *Unified Robot Description Format.* An XML file describing the robot's physical
structure: links (rigid bodies), joints (how links connect and move), joint limits, visual meshes,
and collision meshes. Ours is `src/ur5e_isaac_control/urdf/ur5e.urdf`.

URDF describes *physics and geometry*. It says nothing about intent.

**SRDF** — *Semantic Robot Description Format.* Adds the meaning MoveIt needs on top of the URDF:

- **Planning groups**: which joints form a controllable unit. Ours is `ur_manipulator` — the chain
  from `base_link` to `tool0` (the flange). When you ask MoveIt to plan, you always name a group.
- **Named states**: human-friendly poses, e.g. `home`, `up`.
- **Disabled collision pairs**: pairs of links that are physically adjacent and therefore *always*
  "touching". Without this, the collision checker would report the robot as permanently in
  collision with itself and refuse to plan.

**Planning scene** — MoveIt's model of the world: the robot's current state plus any obstacles
you've registered (conveyor, table, the cube). Collision checking happens against this. Right now
ours contains only the robot; adding the conveyor is milestone M3.

### The three-way split (a very easy thing to get wrong)

A natural but incorrect assumption is that the SRDF, being the "semantic" file, is where the robot
learns about its environment — the table, the conveyor, tags it should detect. **It is not.** The
SRDF never mentions the outside world. It is *entirely about the robot itself*.

If the URDF is the robot's **anatomy**, the SRDF is **annotations on that anatomy**: "these 6
joints together are a thing called `ur_manipulator`", "this pose is called `home`",
"`shoulder_link` and `upper_arm_link` are bolted together, stop reporting them as colliding".

The environment lives somewhere else entirely — a third thing, which is *runtime state* rather than
a file:

| | What it holds | When it's set |
|---|---|---|
| **URDF** | The robot's body: links, joints, limits, meshes | Authored once, static file |
| **SRDF** | The robot's self-knowledge: planning groups, named poses, self-collision pairs | Authored once, static file |
| **Planning scene** | **The world: table, conveyor, the cube being picked** | **Runtime, in `move_group`'s memory — changes as things move** |

Two consequences that shape the remaining milestones:

- **M3 (teaching MoveIt the conveyor exists) is planning-scene code, not SRDF editing.** You
  populate the scene at runtime by publishing collision objects.
- **M1b (adding the gripper) is a URDF change**, because the gripper is part of the robot's *body*,
  not part of the world.

The AprilTag pipeline (`apriltag_pose.py`) is **perception** — it produces a *pose*. That pose is
then inserted into the planning scene as an obstacle to avoid or used as a grasp target. It never
touches the SRDF.

**tool0** — ROS-Industrial's standard name for the bare flange frame at the end of a UR arm. Note
this is the *flange*, not the gripper fingertips — which matters, and is why milestone M1b adds a
`grasp_tcp` frame.

**Action vs Topic** — a topic is fire-and-forget broadcast. An action is a long-running request
with progress feedback, a final result, and the ability to cancel. Executing a trajectory takes
seconds and can fail, so it's an action.

**move_group** — the central MoveIt node. It loads the URDF + SRDF + config, holds the planning
scene, answers planning requests, and dispatches execution. "Running MoveIt" essentially means
"running `move_group`".

---

## 3. Why we could reuse UR's config (and where it fell short)

Universal Robots ships an official MoveIt configuration package, `ur_moveit_config`, already
installed on this system. Because our URDF uses the exact standard UR joint and link names, that
config applies to us directly — a large amount of expert-tuned setup we don't have to write.

| File | Purpose |
|---|---|
| `srdf/ur.srdf.xacro` | The planning group `ur_manipulator`, named states, and self-collision exclusions. |
| `config/kinematics.yaml` | Which IK solver to use for the group. Ours: `KDLKinematicsPlugin`. |
| `config/joint_limits.yaml` | Acceleration limits (5.0 rad/s² per joint). URDF carries velocity/effort limits but *not* acceleration, and MoveIt needs acceleration to compute timing. |
| `config/ompl_planning.yaml` | Selects the OMPL planning library and pre/post-processing adapters. |
| `config/moveit_controllers.yaml` | **The link to our bridge** — see below. |
| `config/moveit.rviz` | A ready-made RViz layout with the Motion Planning panel already set up. |

### The one file that dictates our bridge's design

`moveit_controllers.yaml` declares a controller named `scaled_joint_trajectory_controller`, marked
`default: true`, of type `FollowJointTrajectory`, with action namespace `follow_joint_trajectory`,
covering exactly our six joints. ROS2 composes those into a full action name:

```
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

Our bridge must advertise **exactly** that name, or MoveIt won't find it.

### Why we could NOT use UR's launch file

`ur_moveit_config` ships `ur_moveit.launch.py`, and the obvious move would be to just run it. Two
independent reasons that fails here:

1. It launches a node from the package `ur_robot_driver`, and gates `move_group`/RViz behind that
   node exiting. **`ur_robot_driver` is not installed** — it's the driver for talking to *physical*
   UR hardware over the network, which we deliberately don't want in the loop, so nothing ever
   starts.
2. Its config builder loads **only the SRDF — never a URDF**. It assumes the real-robot driver
   already published the robot description. We have no such driver.

So we write our own launch file that keeps 100% of the config reuse and drops only the
hardware-driver plumbing.

---

## 4. What `launch/moveit.launch.py` does

Starts four nodes:

| Node | Job | What breaks without it |
|---|---|---|
| `robot_state_publisher` | Reads `/joint_states` from Isaac Sim + the URDF, computes where every link *is* in 3D, publishes as TF | RViz can't draw the arm; MoveIt doesn't know the current pose |
| `move_group` | MoveIt itself — planning, scene, dispatch | No planning at all |
| `trajectory_bridge` | Our adapter (Section 5) | Planning works, but **Execute does nothing** |
| `rviz2` | The GUI, with UR's ready-made MoveIt layout | No visuals; planning only via code |

### Planning and executing are separately testable

Launch without the bridge (`launch_bridge:=false`) and RViz will still plan and animate a *ghost*
arm — that's MoveIt working correctly. Only the **real** arm moving proves the bridge works.

So: **Plan succeeds but Execute does nothing → suspect the bridge. Plan itself fails → suspect the
config/URDF/SRDF.**

### Isaac Sim must be running even to *plan*

`move_group` has to know where the arm currently *is* before it can plan a path from there. That
comes from `/joint_states` → `robot_state_publisher` → TF. Isaac Sim is the only source of joint
states in this setup, so with it not running or not playing, RViz will fail to fetch the current
robot state and planning fails before it even starts.

### How the configuration is assembled

```python
moveit_config = (
    MoveItConfigsBuilder(robot_name='ur5e', package_name='ur_moveit_config')
    .robot_description(file_path=urdf_path)                              # OUR urdf
    .robot_description_semantic(Path('srdf') / 'ur.srdf.xacro',
                                {'name': 'ur5e'})                        # UR's srdf
    .robot_description_kinematics(Path('config') / 'kinematics.yaml')    # UR's
    .joint_limits(Path('config') / 'joint_limits.yaml')                  # UR's
    .trajectory_execution(Path('config') / 'moveit_controllers.yaml')    # UR's
    .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
    .to_moveit_configs()
)
```

Read that as: *"take our URDF, and pair it with all of UR's semantic and planning configuration."*
Relative paths resolve inside `ur_moveit_config`; our URDF is passed as an absolute path since it
lives in our own package.

**One subtlety worth knowing:** the SRDF's `<robot name="...">` must match the URDF's
`<robot name="...">`, or MoveIt rejects the pair. UR's SRDF takes the name as an argument, and our
URDF declares `name="ur5e"` — hence `{'name': 'ur5e'}`.

**Harmless warnings you will see:**
- `Cannot infer URDF from .../ur_moveit_config -- using config/ur5e.urdf` — the builder's
  constructor guessing before we override it on the next line; ignore it.
- `No 3D sensor plugin(s) defined for octomap updates` — MoveIt can build a voxel obstacle map from
  a depth camera; only relevant if we later do camera-based obstacle avoidance.

**Cleanup note:** the launch file also carries a small `rviz_env` block that strips
`FASTRTPS_DEFAULT_PROFILES_FILE` from RViz's environment — a leftover from the old
Windows+WSL2 setup where a custom DDS profile broke RViz's request/reply traffic. Now that no such
profile is set anywhere, it's a harmless no-op; worth deleting on the next edit of this file so it
doesn't read as active behavior. The separate block stripping snap-injected GTK/locale env vars
(`GTK_PATH`, `LOCPATH`, etc.) is unrelated and still needed — it works around VSCode's Snap
environment leaking into `rviz2`'s process and pulling in the wrong `libpthread.so.0`.

---

## 5. What `trajectory_bridge.py` does

This is the adapter that fills the gap from Section 1.

### a) The action name is a handshake

MoveIt does not discover controllers by looking around. It reads `moveit_controllers.yaml`, sees a
controller named `scaled_joint_trajectory_controller`, and looks for an action server at exactly:

```
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

Our bridge advertises precisely that string. If it were off by one character, MoveIt would plan
fine and then fail on Execute with *"Unable to identify any set of controllers"*. Worth
recognizing that message — it means a **name mismatch**, not broken execution logic.

### b) What a trajectory actually contains

A planned 3-second motion arrives as roughly 30 points:

```
point[0]:  positions=[0.00, -1.57,  0.00, -1.57,  0.00, 0.00]   time_from_start=0.00s
point[1]:  positions=[0.04, -1.55,  0.03, -1.56, -0.01, 0.00]   time_from_start=0.11s
point[2]:  positions=[0.09, -1.52,  0.07, -1.55, -0.03, 0.00]   time_from_start=0.23s
...
point[29]: positions=[0.80, -1.00,  1.00, -1.50, -1.57, 0.00]   time_from_start=3.00s
```

Note the timestamps are **not evenly spaced**, and the motion starts slow, speeds up, then slows
down. That is MoveIt respecting acceleration limits: it planned a *velocity profile*, not merely a
path.

### c) The core trick: sample by elapsed time

Isaac Sim wants a smooth stream of "go here now". So 60 times a second the bridge asks *how long
since I started?*, finds the two waypoints bracketing that instant, and linearly interpolates
between them.

At t = 0.17 s — between point[1] (0.11 s) and point[2] (0.23 s), 50% through that segment — it
publishes the midpoint of those two poses.

**Why not just replay the 30 points in order?** That would discard the timing. MoveIt carefully
worked out *when* each point should be reached; replaying at our own fixed rate produces a
different, possibly jerky or limit-violating motion. Sampling against the wall clock reproduces
MoveIt's intended profile.

Steps in code: reject empty trajectories; loop at 60 Hz sampling and publishing to `/joint_command`
as a `JointState`; publish action feedback each tick containing desired position, actual position
(from `/joint_states`), and the error between them.

### d) One honest simplification

Each trajectory point also carries **velocities and accelerations**, and our bridge **ignores
them** — it sends positions only. That's the correct call here, because the Action Graph wires
`/joint_command` into the Articulation Controller's *Position Command* input (Section 6); Isaac
Sim's PD drives take position targets. But it's a real simplification: we drop velocity
feedforward, which on real hardware improves tracking. **If motion looks laggy under fast conveyor
picks (M5), this is the first thing to revisit.**

### e) Why the goal check uses a tolerance

Because those PD drives *track* a target with a small lag rather than snapping to it, the arm never
lands exactly on the final commanded angle. After the trajectory's duration elapses the bridge
pauses briefly, then compares actual joint positions against the final target with a tolerance
(default 0.05 rad, about 2.9°). Demanding exactness would make every goal report failure despite
the arm having moved correctly.

The node also warns if nothing is subscribed to `/joint_command` (Isaac Sim not playing → commands
silently vanish), and reads current joint positions from `/joint_states` rather than assuming a
starting pose.

### Testing the maths without a robot

The interpolation lives in a plain function, `sample_trajectory(points, t)`, deliberately kept free
of ROS types so it can be tested directly. Verified: correct midpoint interpolation, clamping
before the start and after the end, and no division-by-zero on a zero-duration segment.

---

## 6. Isaac Sim side: the Action Graph

Built via `Create -> Visual Scripting -> Action Graph` (node search names in parentheses):

```
On Playback Tick --exec--> Isaac Read Joint State Node --exec--> ROS2 Publish Joint State
                 \-exec--> ROS2 Subscribe Joint State --exec--> Articulation Controller

Isaac Read Joint State Node --data--> ROS2 Publish Joint State
  (Joint Names, Joint Positions, Joint Velocities, Joint Efforts,
   Joint Dof Types, Sensor Time, Stage Meters Per Unit -> matching inputs)

ROS2 Subscribe Joint State --data--> Articulation Controller
  (Joint Names, Position Command, Velocity Command, Effort Command
   -> matching inputs)

ROS2 Context --> Context input on both ROS2 Publish/Subscribe Joint State nodes
```

- `Isaac Read Joint State Node` → **Prim Path**: the actual articulation root.
- `Articulation Controller` → **Robot Path**: same path, as a plain string.
- `ROS2 Publish Joint State` topic: `joint_states` (default).
- `ROS2 Subscribe Joint State` topic: `joint_command` (default).

### Gotcha: the articulation root wasn't where expected

Console errors (`Failed to find articulation at '/World/ur5e'`,
`Pattern '/World/ur5e' did not match any articulations`) revealed the real articulation root API
was applied to a child prim, **`root_joint`**, not the top-level `/World/ur5e` Xform — a known quirk
of the URDF importer when "Fix Base" is enabled. Found the real path in
`Window -> Script Editor` rather than hunting through the stage tree:

```python
import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        print(f"Articulation root: {prim.GetPath()}")
```

### Gotcha: "Simulation view object is invalidated" — commands silently do nothing

Once the ROS2 connection and OmniGraph wiring were confirmed correct (real data visibly flowing
into `Articulation Controller`'s inputs), the arm *still* didn't move:

```
[omni.physx.tensors.plugin] Simulation view object is invalidated and cannot be used again
[omni.graph.core.plugin] .../articulation_controller: OmniGraph Error: Articulation
  controller failed for prim '/World/ur5e/root_joint': Failed to get DOF position targets
```

`Articulation Controller` (and joint-state nodes) cache a handle into the PhysX tensor API, which
gets invalidated when the physics scene is torn down/rebuilt — e.g. from editing the graph or stage
*while* Play is active. Fix: **Stop**, then **Play** again — a clean stop/play cycle.

**Make Action Graph / stage edits while stopped, then press Play fresh**, to avoid retriggering
this.

---

## 7. What's built on the ROS2 side

### `joint_command_publisher`

- Subscribes to `/joint_states` to get the arm's current position before moving (avoids a jump from
  a stale default).
- Waits for an actual subscriber match on `/joint_command` before publishing (a one-shot publish
  before discovery completes gets silently dropped).
- Linearly interpolates from current position to a target pose and streams it at 60 Hz over a given
  duration.

```bash
# Move to an explicit target pose (radians, 6 comma-separated values)
ros2 run ur5e_isaac_control joint_command_publisher \
  --positions "0,-1.2,1.2,-1.5,-1.57,0" --duration 3

# Preset two-pose sweep, for a quick sanity check
ros2 run ur5e_isaac_control joint_command_publisher --demo --duration 3

# Preset pick-and-place mimic (approach/pick/lift/transfer/place/lift/home)
ros2 run ur5e_isaac_control joint_command_publisher --pick-place
```

The joints have real position-drive gains, so Isaac Sim's PD controller **holds the last commanded
position** on its own — no need to publish continuously to keep the arm in place.

### `joint_controller_gui`

A Tk slider panel — one slider per UR5e joint — for manually driving the arm and learning its
joint ordering, limits, and directions before scripting motions.

- Subscribes to `/joint_states` and **snaps the sliders to the arm's real pose on startup** (and
  via **Sync to current**), so enabling publish never yanks the arm from a stale value.
- Streams the live slider pose to `/joint_command` at 30 Hz while **Publish** is checked.
- Sliders span the UR5e's full ±2π range with a live radians/degrees readout. **Home** sends the
  default pose.
- Status line shows publish on/off and the live `/joint_command` subscriber count.

```bash
# Isaac Sim must be playing first
ros2 run ur5e_isaac_control joint_controller_gui
```

### `kinematics.py`

ikpy-based FK/IK on the official `ur_description` URDF chain, `base_link → … → tool0`.

| Method | Description |
|---|---|
| `fk(q6)` | 6 joint angles (rad) → 4×4 TCP pose in `base_link` |
| `ik(target_pose, initial_q6)` | 4×4 TCP target → 6 joint angles (rad) |
| `pose_error(q6, target)` | (position error [m], orientation error [rad]) |
| `tcp_z_offset` (ctor) | Metres from `tool0` to the grasp point along tool +Z (gripper length) |

Validated: 200 random reachable configurations, round-trip `IK(FK(q)) → q'`, max position error
0.0000 mm, max orientation error 0.0000°, 0 failures. FK sanity: reach at zero pose = 0.852 m
(matches UR5e's 0.85 m spec).

### `apriltag_pose.py`

Detects 36h11 AprilTags with `cv2.aruco` and recovers 6-DOF pose with `cv2.solvePnP`
(`SOLVEPNP_IPPE_SQUARE`, optimal for planar squares — `estimatePoseSingleMarkers` was removed in
OpenCV 4.7+, so pose is computed from known metric corner geometry, the current standard method).

| Method | Description |
|---|---|
| `detect(image, K, dist)` | → list of `{id, corners, T_cam_tag, rvec, tvec}` |
| `pose_from_corners(pts, K, dist)` | 4 tag corners → `(T_cam_tag, rvec, tvec)`; decoupled for testing |
| `draw_detections(...)` | Debug overlay: tag outline, axes, id |

`T_cam_tag` expresses the tag frame in the camera optical frame (OpenCV convention).

**Works on OpenCV 4.6 and 4.7+.** OpenCV reorganised aruco in 4.7 — `ArucoDetector` and
`DetectorParameters()` replaced the free `detectMarkers()` and `DetectorParameters_create()`. This
file originally used only the 4.7 API; Ubuntu 24.04 / ROS 2 Jazzy ship **4.6.0**, where the new
class does not exist and the new-style constructor **segfaults**. Fixed 2026-08-11 with a
`_HAS_ARUCO_DETECTOR` branch.

**Correction to an earlier claim.** This file was previously described here and in PLAN.md as
"validated — 50 poses, 0.0002 mm error". That validation exercised only `pose_from_corners` (the
`solvePnP` maths, which works fine on 4.6). The **detection** path had never run on this machine —
it crashed the first time it was called. Detection is now verified live (2026-08-11): tag id 0 off
the Isaac Sim camera at 56.8 px, lateral position accurate to **0.05 mm**, range 2.0% long. See
§11.

Tags generated: 36h11, ids 0–3, 1040×1040 px with a white quiet-zone border (required for
detection); the **inner 800 px black square is the metric `tag_size`** — ratio 800/1040 = 10/13 =
0.769. Getting this wrong is a pure scale error on every distance the detector reports.

### `cube_pose_publisher`

The perception node. Turns `/rgb` + `/camera_info` into the cube's pose in coordinates MoveIt can
plan to.

```bash
ros2 run ur5e_isaac_control cube_pose_publisher --ros-args -p use_sim_time:=true
```

| Publishes | |
|---|---|
| `/cube_pose` | `geometry_msgs/PoseStamped` in `base_link` — the cube **centre**, yaw-only orientation |
| `/cube_marker` | `visualization_msgs/Marker` for RViz |
| TF | static `base_link → sim_camera`, from the extrinsic, so everything downstream agrees |

Every number is a ROS parameter — `tag_size`, `tag_id`, `extrinsic_xyz`, `extrinsic_quat`,
`tag_z_base`, `tag_to_cube_z`, `cube_size`, `normalize_yaw_90`. Moving the camera or the robot
means re-running `scripts/isaac_camera_extrinsic.py` and changing a parameter, not editing code.

Three decisions inside it that are not obvious, all explained in §11:

- **Depth is asserted, not measured** — corrected by ray–plane intersection against `tag_z_base`.
- **Roll and pitch are forced to zero**; only yaw survives.
- **Yaw is folded into ±45°**, since a cube looks identical every 90°, so all four grasp angles are
  equivalent and this keeps the wrist near zero.

Verified 2026-08-11 against USD ground truth: **0.2 mm in X, 0.3 mm in Y, exact in Z**, stable to
four decimal places across 200 consecutive detections, at ~17 Hz.

---

## 8. Everyday restart checklist

1. Launch Isaac Sim (`~/isaacsim/isaac-sim.sh`), open the scene, press **Play**.
2. If you edited the graph/stage since the last Play, do one **Stop → Play** cycle first (the
   simulation-view invalidation gotcha, Section 6).
3. `ros2 topic list` should show `/joint_states`, `/joint_command` and `/clock` within a few
   seconds — no DDS profile or env var to double-check anymore now that both sides are on the same
   OS. With the camera graph running you should also see `/rgb` and `/camera_info`.
4. Run `joint_command_publisher`, `joint_controller_gui`, or `ros2 launch ur5e_isaac_control
   moveit.launch.py` as needed. Add `launch_rviz:=false` to skip RViz when you don't need it —
   8 GB VRAM is tight once the camera render product is also live.
5. To see what the camera sees: `ros2 run rqt_image_view rqt_image_view /rgb`. (If it dies with a
   `GLIBC_PRIVATE` / `libpthread` error, that is the VSCode-snap GUI conflict — run it from a plain
   terminal. Same cause as the env-stripping block in `moveit.launch.py`.)
6. For the cube's pose: `ros2 run ur5e_isaac_control cube_pose_publisher --ros-args
   -p use_sim_time:=true`, then `ros2 topic echo /cube_pose`.

**If you move the camera, the robot base, or the belt height**, re-run
`scripts/isaac_camera_extrinsic.py` and update `cube_pose_publisher`'s `extrinsic_*` and
`tag_z_base` parameters. Those numbers describe a *relationship*, so moving either end invalidates
them.

**Operating rule:** if you Stop and re-Play Isaac Sim, **restart the ROS launch too.** Sim time
resets to 0 and nodes that already saw t=1657 will see time jump backwards.

---

## 9. What has been verified, and what has not

**Verified (OS/transport-independent — still holds):**

- MoveIt core, `ur_moveit_config`, `moveit_servo`, RViz plugin, and the `control_msgs` action types
  are all installed.
- Our URDF pairs cleanly with UR's SRDF; the group, IK solver, and controller list resolve
  correctly in the assembled config.
- `move_group` starts and reports **"You can start planning now!"**.
- The bridge advertises `/scaled_joint_trajectory_controller/follow_joint_trajectory`.
- Trajectory interpolation maths (`sample_trajectory`) — ad-hoc, **not** committed as a test; see
  ISSUES.md.
- `kinematics.py` FK/IK round-trip, and `apriltag_pose.py`'s **pose maths** (Section 7 numbers).
  The FK/IK half was re-run on this machine 2026-09-04 once `ikpy` was installed: `tool0` at
  0.8521 m at the zero pose, 200-pose IK round-trip 0.0000 mm / 0.0000°.
- Live joint state streaming and commanded-position control of the real Isaac Sim arm via
  `joint_command_publisher` / `joint_controller_gui`, including the pick-and-place mimic sequence
  and gripper commanding.

**Verified live 2026-08-10 (the end-to-end chain):**

- MoveIt **planning and `trajectory_bridge` executing onto the real Isaac Sim arm**. RViz Plan
  succeeds in 5–25 ms; Execute moves the arm, `SUCCEEDED`, worst joint error 0.0000–0.0001 rad.
- M1b: gripper collision geometry and planning to `grasp_tcp` across the reachable workspace.

**Verified live 2026-08-11 (perception):**

- Isaac Sim publishing `/rgb` + `/camera_info` over the ROS 2 bridge, ~20 Hz, sim-time stamped.
- `apriltag_pose.py` **detecting** a tag in a live Isaac Sim frame (this had never run before —
  see Section 7's correction), and the resulting position checked against USD ground truth.
- OMPL planning large joint-space goals (17 ms, 8 ms) — the "large drags fail" worry.
- The camera→`base_link` extrinsic, and `cube_pose_publisher` end to end: cube position in
  `base_link` accurate to 0.2 mm / 0.3 mm / exact, stable over 200 detections.
- Reach, empirically: the arm was commanded straight out at the cube and observed to fall short,
  which is what prompted moving the scene. Now 0.745 m against a 0.850 m limit.

**Not yet verified:**

- **Whether USD's `base_link` is yawed relative to the URDF's.** Both the extrinsic and the ground
  truth it was graded against come from USD, so they share the assumption and cannot detect it.
  Settle before M4 (ISSUES.md says how).
- Anything involving the planning scene (M3) or an actual grasp (M4).

---

## 10. Bringing up the stack, and reading the failures

✅ **This sequence passed on 2026-08-10** — MoveIt plans and the real Isaac Sim arm executes. Kept
here as the standard bring-up procedure and, more usefully, as a map from symptom to cause.

**Prerequisites, in order. Do not skip the check at step 2.**

1. **Isaac Sim:** open the scene, press **Play**. If the stage or Action Graph was edited since the
   last Play, do a clean **Stop → Play** cycle first (Section 6).

2. **Verify the joint bridge is alive before involving MoveIt:**
   ```bash
   ros2 topic echo /joint_states
   ```
   Data should stream. If nothing appears, stop — the problem is the Isaac Sim Action Graph wiring
   (Section 6), not MoveIt.

3. **Launch MoveIt:**
   ```bash
   cd ~/isaacsim_ros_ws
   source install/setup.bash
   ros2 launch ur5e_isaac_control moveit.launch.py
   ```
   RViz opens with the Motion Planning panel.

4. **Plan:** drag the interactive marker at the arm's tip to a new pose, click **Plan**. A
   semi-transparent *ghost* arm should animate the proposed path.

5. **Execute:** click **Execute**. The **real Isaac Sim arm** should move.

**Interpreting the outcome:**

| Result | Where the problem is |
|---|---|
| Step 4 fails ("failed to fetch current robot state") | Isaac Sim not playing, or `/joint_states` not arriving |
| Step 4 fails (no plan found) | Config, URDF/SRDF, or the goal pose is unreachable |
| Step 4 works, step 5 does nothing | The bridge — check its terminal output |
| Step 5 errors "unable to identify any set of controllers" | Action name mismatch (Section 5a) |
| Both work | **M1 + M2 confirmed** — as they were on 2026-08-10. |

The RViz Motion Planning panel is dense and not obvious on first use — worth walking through its
tabs before clicking, rather than hunting for the buttons.

**Historical note:** `move_group` used to log `Didn't receive robot state (joint angles) with
recent timestamp within 1.000000 seconds`, because Isaac Sim stamps `/joint_states` with simulation
time while `move_group` compared it against wall-clock. Planning tolerated it; **execution did
not**, and that is why M2 could not be verified for a long time. Resolved 2026-08-10 by publishing
`/clock` from the Action Graph and setting `use_sim_time: true` on all four nodes. If you see this
warning again, `/clock` has stopped — check Isaac Sim is playing.

**Benign warnings (safe to ignore):**
- `SEVERE WARNING!!! ... namespace collision ... InteractiveMarkerDisplay` — cosmetic RViz2/Jazzy
  plugin double-registration.
- `Publisher already registered for node name: 'rviz2_moveit'` — duplicate logger registration, no
  functional effect.
- `Action server: /recognize_objects not available` — unused feature.

---

## 11. Perception: pixels to a pose

Built and verified 2026-08-11. The exact numbers live in PLAN.md; this is what the pieces *do*.

### Why a tag at all

A plain grey cube gives a camera almost nothing: it's texture-less, its silhouette from above is a
square whichever way it's turned, and a small cube nearby looks identical to a big one far away.

An AprilTag fixes all three by being a **known, asymmetric, high-contrast pattern of known physical
size**: the known size resolves scale, the asymmetric pattern resolves rotation, the encoded ID
says *which* object this is. "36h11" means 36 data bits (a 6×6 grid) with a minimum Hamming
distance of 11 between valid codes — up to 5 bits can be misread and it still decodes correctly, or
not at all. It never quietly returns the wrong ID.

### The camera model, and where the numbers come from

Everything rests on the pinhole model: a point at `(X, Y, Z)` in the camera's optical frame lands at
pixel `u = fx·X/Z + cx`, `v = fy·Y/Z + cy`. Those four numbers are the **intrinsics**, published on
`/camera_info`.

A USD camera is described like a physical one — `focalLength` and `horizontalAperture` — and the
bridge converts: `fx = width_px × focalLength / horizontalAperture`. Both are in the same units so
the ratio is unitless, which makes this correct regardless of stage units. Field of view falls out
of the same two numbers, so "zoom in" and "raise focal length" are the same operation.

Two things must be right or the image is quietly wrong:

- `verticalAperture` must equal `horizontalAperture × height/width`, or `fx ≠ fy`, the image is
  stretched, and anything measured from it skews. **Set it from a script, not the Property panel** —
  the panel adjusts these fields as you type and three attempts by hand never converged.
- The projection must be **perspective**. See ISSUES.md; this one cost a day.

We publish `/camera_info` rather than hardcoding the intrinsics so that changing the lens or
resolution can't leave a stale copy behind.

### Scale is the whole game

`solvePnP` takes the tag's four corners in pixels, the same four corners in metres (from
`tag_size`), and the intrinsics, and solves for the rotation and translation that project one onto
the other.

But a monocular camera measures **angles only**. A small tag close and a large tag far produce
pixel-identical images. The single thing that breaks the tie is your assertion of the tag's true
size, and the relationship is linear:

> **Tell the solver the tag is 30% bigger than it is, and it reports the cube 30% further away.**
> No error, no warning.

Which is why the quiet zone matters. The PNGs are 1040 px with the black pattern occupying the
inner 800 px. If you map the whole PNG onto a 0.045 m quad, the *black square* is 0.0346 m — pass
0.045 and every distance is 1.3× too far. Keep the white margin in the texture (the detector needs
it to find the tag's outer edge) and feed the solver `quad × 0.769`.

### The pixel budget

The tag is 8 cells across, and decoding samples each cell, so you need several pixels per cell.
Below that the bits alias, the Hamming budget is exhausted, and the tag is **rejected silently** —
which looks exactly like a broken pipeline. `scripts/isaac_camera_check.py` predicts this from the
lens and the geometry before anything is built. Measured thresholds are in [ISSUES.md](ISSUES.md).

### Which numbers to trust

Measured against USD ground truth, the detector's **lateral position was accurate to 0.05 mm** and
its **range was 2.0% long**. That split is structural, not bad luck:

- Lateral position is read directly off where the tag sits in the image — well conditioned.
- Depth is inferred *entirely* from apparent size, so any bias in corner localisation lands on it.
  At 14:1 texture minification, mipmap blur makes the detector read the tag ~1.1 px small, and that
  1.9% becomes 2.0% of range.

The same reasoning applies to orientation: viewing a flat square nearly head-on makes the
out-of-plane tilt degenerate — two mirror solutions fit almost equally well — while the in-plane
rotation stays well determined. So: **take X, Y and yaw from vision; take depth and tilt from known
geometry.** Replacing an ill-conditioned measurement with a known constraint is the correct move,
not a shortcut.

### Fixing depth properly: slide along the ray, don't overwrite Z

The obvious implementation — keep vision's X and Y, replace Z with the known height — is **not
good enough**, and the reason is worth understanding because it is invisible in the easy case.

Lateral position comes out of the pinhole model as

```
X = (u - cx)/fx · Z
```

It is **proportional to the depth estimate**. So a depth bias does not stay in Z: it leaks
sideways, in proportion to how far off the optical axis the tag sits.

- Tag directly under the camera → no leak. (An early test in exactly this position measured
  0.05 mm laterally, which flattered the method and hid the problem.)
- Tag 7° off-axis → **+4.5 mm in X, −5.1 mm in Y**, alongside −55.5 mm in Z.

The fix follows from what is actually trustworthy. The *direction* from the camera to the tag is
well determined; only the *distance along it* is biased. And the tag's height is known, because the
cube sits on a flat belt. So intersect the ray with that plane:

```
ray   = p_tag_detected − camera_origin        (both in base_link)
scale = (tag_z_known − camera_z) / ray_z
p     = camera_origin + scale · ray
```

One line, and it corrects all three axes at once. Measured result: **0.2 mm, 0.3 mm, exact** — and
Z stops jittering entirely, because it is no longer being measured.

> This is the same idea that makes cheap colour-blob detection viable without any tag at all: a
> camera gives you a *ray*, not a point, and something else has to supply the distance. An AprilTag
> supplies it via known size (fragile — it is the axis every bias lands on). A known ground plane
> supplies it exactly. When both are available, prefer the plane.

### Frames, and the rotation everyone forgets

`apriltag_pose.py` returns the tag in the **OpenCV optical frame** (X right, Y **down**, Z **out of
the lens**). MoveIt wants it in `base_link`. That is a three-link chain, and the middle link is the
one that gets skipped:

```
T_base_tag  =  T_base_camlink · T_camlink_optical · T_optical_tag
```

ROS's `camera_link` convention is X forward, Y left, Z up — a fixed 90°/90° rotation from the
optical frame. Omit it and the cube lands at a plausible distance in entirely the wrong direction.

`T_base_camlink` is a **constant**, because the camera is bolted to the world rather than the
wrist. That was the point of choosing eye-to-hand: measure once, and the camera keeps watching the
belt while the arm is busy. `scripts/isaac_camera_extrinsic.py` reads it off the USD stage and
folds in one more flip most people miss: **a USD camera looks down −Z with +Y up, a ROS optical
frame looks down +Z with +Y down** — 180° apart about X. Skip that and the tag lands *behind* the
camera.

That script sidesteps a second trap too. USD matrices are **row-vector** (`p_world = p_local · M`),
so the rotation block is the transpose of what ROS expects. Rather than transpose by hand and hope,
it asks USD to transform the camera's own axis directions and builds the rotation from the results
— correct by construction. It prints the viewing direction as a built-in check: for a top-down
camera it should read `(0, 0, −1)`.

⚠️ Still note [ISSUES.md](ISSUES.md)'s warning that USD link-frame *orientations* do not match the URDF's. The
extrinsic and any USD-derived "ground truth" share that assumption, so agreement between them does
not rule out a yaw offset against the URDF's `base_link`. Confirm visually in RViz.

---

## 12. Concepts still ahead

Deliberately not covered yet; each arrives with its milestone.

- **Collision objects / planning scene** (M3) — teaching MoveIt that the conveyor and table exist.
- **Cartesian vs joint-space goals** — "move in a straight line" vs "get there however you like".
- **Predict-and-intercept** (M5) — why picking a *moving* object isn't solved by re-planning
  continuously, and what is done instead.
- **MoveIt Servo** (M6) — continuous velocity streaming for real-time tracking, bypassing
  plan-then-execute entirely.
