# MoveIt2 Conveyor-Pick Project Plan (UR5e + Robotiq 2F-85, Isaac Sim <-> ROS2 Jazzy)

Written for zero prior MoveIt experience. Builds on the working Isaac Sim <->
ROS2 Jazzy bridge in `docs/SETUP_NOTES.md` — that part is done and unchanged
by anything below.

## 0. What MoveIt actually is (concepts primer)

MoveIt is a **motion planning** framework. It does NOT talk to your robot
directly. It only:

1. Knows the robot's kinematic structure (from URDF) and which joints form a
   controllable "planning group" (from an SRDF — e.g. `ur_manipulator` for
   the 6 arm joints, `gripper` for the 2F-85).
2. Given a goal (a Cartesian pose, or a named joint configuration), computes
   IK and a smooth, **collision-checked** joint trajectory to get there —
   checking against a "planning scene" (the robot's own links, plus any
   obstacles you register, e.g. the conveyor rails, the table).
3. Hands that trajectory to an **execution** layer, which sends it to
   whatever actually moves the robot, via the ROS2-standard
   `control_msgs/action/FollowJointTrajectory` action interface — the same
   interface `ros2_control`'s `joint_trajectory_controller` normally
   provides for a real/simulated robot.

This is the piece you don't have yet. Right now, "motion" in this repo means
either manually jogging sliders or streaming a hardcoded joint list — no
planner, no collision checking, no goal-directed IK against a scene. MoveIt
doesn't replace your Isaac Sim bridge; it sits on top of it and needs a
small adapter to talk to it (Section 2, M2).

**Why a moving conveyor item isn't handled by continuous re-planning:** a
single MoveIt plan typically takes 50-200ms+ to compute — too slow to
re-plan every frame against a live-moving target. The standard industrial
approach (and what M5 below does) is **predict-and-intercept**: compute
where the item *will* be at a chosen future time (from known belt speed),
plan once to a static predicted pose, and time execution so the arm arrives
exactly when the item does. True continuous visual tracking (MoveIt Servo,
velocity-streamed, replanning-free) is a later upgrade (M6) if prediction
alone isn't accurate enough.

## 1. Target architecture

```
Isaac Sim (Windows)                          WSL2 (ROS2 Jazzy)
--------------------                          ------------------
UR5e + Robotiq 2F-85          /joint_states    move_group (ur_moveit_config)
articulation(s)          <=================>       ^
  ROS2 Publish/Subscribe        (DDS, unicast        | plan requests
  Joint State (existing)        loopback profile)    |
        ^                                     pick_orchestrator node (new)
        |  /joint_command, /gripper_command      - state machine
        |  (FollowJointTrajectory bridge, new)    - talks to move_group via
        |                                           MoveGroupInterface (Python)
        +--- trajectory_bridge node (new) <---------+
                                                      |
                                             conveyor_kinematics node (new)
                                               - spawn time + belt speed
                                               -> predicted cube pose/time
                                               (later: swap in apriltag_pose.py
                                                for real perception)
```

RViz + the MoveIt "Motion Planning" plugin sits alongside `move_group` for
visualizing/debugging plans — not required at runtime once the orchestrator
is scripted, but essential while building each milestone.

## 1b. Environment verification (checked 2026-08-07, WSL2 Jazzy)

Verified directly against the installed system rather than assumed.

**Already present — no install needed:**

| Component | Status |
|---|---|
| `moveit_ros_move_group` (the `move_group` node) | installed |
| `moveit_planners_ompl` / `chomp` / `stomp` | installed |
| `moveit_ros_visualization` (RViz Motion Planning plugin) | installed |
| `moveit_simple_controller_manager` | installed |
| `moveit_servo` (needed only for M6) | installed |
| `ur_moveit_config` 3.8.0 | installed |
| `ur_description` 3.5.1 (supplies the URDF meshes) | installed |
| `control_msgs` (FollowJointTrajectory), `joint_trajectory_controller` | installed |
| `robot_state_publisher`, `xacro`, `warehouse_ros_sqlite` | installed |
| `ikpy` 4.0.0, OpenCV 4.12.0 | installed |

**Missing / to install:**

- `ros-jazzy-moveit-py` — the Python planning API (`from moveit.planning import
  MoveItPy`) is **not** installed; `import moveit` currently fails. Required
  for the orchestrator in M4/M5 unless we drive the `/move_action` action
  interface manually.
- `ros-jazzy-ur-robot-driver` — **deliberately not installing** (see below).
- `moveit_setup_assistant` — not installed; moot, since we're reusing config.

**Repo URDF is compatible.** `src/ur5e_isaac_control/urdf/ur5e.urdf` uses the
exact official UR joint/link names the SRDF and `moveit_controllers.yaml`
expect, carries proper `<limit>` tags on all 6 joints, and its
`package://ur_description/meshes/ur5e/...` references resolve against the
installed `ur_description`. It has no `ros2_control` tags — which is correct
here, since Isaac Sim *is* the hardware.

### Blocker: `ur_moveit.launch.py` can't be used as-is

Two independent reasons, both confirmed by reading the installed launch file:

1. It launches `Node(package="ur_robot_driver", executable=
   "wait_for_robot_description")` and gates `move_group` + RViz behind that
   node's `OnProcessExit`. `ur_robot_driver` is not installed, so nothing
   ever starts.
2. Its `MoveItConfigsBuilder` sets **only** `.robot_description_semantic(...)`
   — it never loads a URDF at all. It assumes `ur_control.launch.py` from the
   real-robot driver already published `robot_description`.

Installing `ur_robot_driver` would fix (1) but not (2), and it exists to talk
to physical UR hardware over the network — not something we want in the loop.
**Decision: write our own launch file** that runs `robot_state_publisher` with
our URDF plus `move_group` parameterised from `ur_moveit_config`'s existing
SRDF/kinematics/OMPL/joint-limits YAML. We keep 100% of the config reuse and
drop only the driver-specific plumbing.

### Verified: the exact controller name the bridge must advertise

`ur_moveit_config/config/moveit_controllers.yaml` declares
`scaled_joint_trajectory_controller` with `default: true`, `type:
FollowJointTrajectory`, `action_ns: follow_joint_trajectory`, over exactly our
6 joint names. So the M2 bridge node must advertise the action

```
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

(or we override that YAML with our own controller name — but matching it is
free, so we match it.)

### Verified: the official SRDF has no gripper

`ur_macro.srdf.xacro` defines a single group, `ur_manipulator`
(`base_link` -> `tool0`), three group states (`home`, `up`,
`test_configuration`), and self-collision exclusions. There is **no gripper
group and no `<end_effector>` element** — and the URDF has no gripper links
either. So MoveIt currently cannot see the 2F-85 for collision checking, and
plans to `tool0` (the bare flange), not to the finger tips.

**Approach (pragmatic, industrial-standard):** treat the gripper as *geometry*
to MoveIt but not as a *planning group*.

- Extend the URDF with the 2F-85 as fixed collision geometry on `tool0`, plus
  a `grasp_tcp` fixed frame at the finger tips. MoveIt then avoids collisions
  with the gripper body and can plan directly for the real grasp point.
- Control the fingers open/close through a **separate** ROS2 interface (M0),
  not through `move_group`. Two mimic-jointed fingers don't need motion
  planning, and adding a second planning group + SRDF end-effector is
  meaningful extra complexity for no gain at this stage.

## 2. Milestones (revised after verification)

**M1 — Custom launch: `robot_state_publisher` + `move_group` + RViz.**
Write `ur5e_isaac_control/launch/moveit.launch.py` building a
`MoveItConfigsBuilder` from our URDF + `ur_moveit_config`'s SRDF and YAML
(per the blocker above). Run with MoveIt's own fake controller — no Isaac Sim
yet. Proof: RViz Motion Planning panel appears, drag the interactive marker,
Plan succeeds, the *ghost* arm animates. Confirms URDF/SRDF/IK/self-collision
are all sound in isolation. Also `apt install ros-jazzy-moveit-py` here.

**M0 — Gripper ROS2 wiring.**
The 2F-85 is controllable in the Isaac Sim scene (verified previously), but
that path isn't in this repo — confirmed zero gripper references in
`ur5e_isaac_control`. Find its articulation/joint names in the USD stage
(same Script Editor traversal trick as SETUP_NOTES.md used for the
articulation root), expose it over ROS2 via its own Action Graph branch, and
jog it manually to confirm. Independent of M1; do whichever is convenient.

**M1b — Extend the URDF with gripper geometry + `grasp_tcp` frame.**
Per the SRDF finding. Small xacro/URDF edit; lets MoveIt plan to the actual
grasp point and avoid driving the gripper body through the conveyor. Also
lets `kinematics.py`'s existing `tcp_z_offset` argument finally be given a
real measured value.

**M2 — The FollowJointTrajectory bridge node.**
The one substantial piece of new code: a Python node advertising
`/scaled_joint_trajectory_controller/follow_joint_trajectory` (name verified
above), consuming MoveIt's `JointTrajectory` and streaming it to
`/joint_command` — reusing the interpolation and the discovery-wait /
no-stale-jump precautions already proven in `joint_command_publisher.py`.
Must honour trajectory point `time_from_start` and publish action feedback.
Proof: Plan + Execute in RViz moves the **real Isaac Sim arm**.

**M3 — Planning scene collision objects.**
Register the conveyor rails, belt surface, and table as collision geometry so
plans route around them. Sourced from the same measurements the spawner uses.

**M4 — Fixed-point pick-and-place (first full-loop success).**
`pick_orchestrator` node: approach a **known, fixed** pick pose -> close
gripper -> lift -> transfer -> place -> open -> home. Proves the entire chain
(planning, execution, gripper) before adding motion.

**M5 — Dynamic intercept (the actual target).**
A `conveyor_kinematics` node computes the cube's live pose and picks an
intercept point + arrival time; the orchestrator plans once to that predicted
pose, timed to finish on arrival.

> **Correction to the original plan:** belt velocity is *not* available in
> `isaac_conveyor_spawner.py` — that script only defines `SPAWN_POS`,
> `DESPAWN_Y`, and spawn timing. The cubes are moved by the **conveyor prim's
> PhysX surface velocity**, set on the USD stage in Isaac Sim. So M5 must
> either read that value off the prim, or (better) have the spawner record
> each cube's spawn wall-clock time and publish it, letting the ROS2 side
> derive speed empirically from observed cube motion. The empirical route also
> self-corrects for belt slip.

**M6 — Closed-loop tracking (stretch).**
Only if M5's open-loop prediction proves insufficient: feed
`apriltag_pose.py`'s live pose stream into `moveit_servo` (already installed,
and `ur_moveit_config` even ships `config/ur_servo.yaml`) for continuous
velocity-based tracking.

## 3. Order of work

```
M1 ──┬── M1b ──┐
     │         ├── M2 ── M3 ── M4 ── M5 ── M6
M0 ──┴─────────┘
```

Recommend **M1 first** — it's now purely a launch-file exercise, needs no
Isaac Sim, and seeing MoveIt actually plan builds the mental model that makes
every later milestone easier to reason about.
