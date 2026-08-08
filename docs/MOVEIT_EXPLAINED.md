# MoveIt2, Explained From Zero (for this Isaac Sim UR5e project)

A companion to `MOVEIT_PROJECT_PLAN.md`. The plan says *what* we're building
and in what order; this file explains *why each piece exists* and what the
concepts mean, assuming no prior MoveIt or motion-planning knowledge.

Written as the project progresses, so it tracks what has actually been built
and verified — not generic tutorial material.

---

## 1. The core concept: MoveIt does not move robots

This sounds absurd, but it is the single idea the whole architecture follows
from.

**MoveIt is a planner, not a driver.** Think of it as a GPS: it computes a
route, but it never touches the steering wheel.

You give MoveIt a goal — "put the gripper *here*" — and it computes:

- **Inverse kinematics (IK)**: which 6 joint angles place the gripper at that
  pose. (Usually several answers exist; the solver picks one.)
- **A collision-free path**: a sequence of intermediate joint positions that
  gets from the current pose to the goal without driving the arm through the
  conveyor, the table, or itself.
- **Timing**: how fast to move through each point without exceeding the
  joints' velocity and acceleration limits.

The output is a **trajectory** — a list of waypoints, each of the form
*"at t = 0.7 s, the six joints should be at these angles."*

MoveIt then hands that list to something else to actually execute it.

A useful shorthand: **MoveIt is a backend calculator, not a driver.**

Two refinements to that shorthand, both of which matter later:

- `move_group` is not purely passive. It also **dispatches and monitors**
  execution — it sends the trajectory out and waits to hear whether it
  succeeded. Extending the GPS analogy: it computes the route *and* says "turn
  now"; it just can't physically turn the wheel. This is why our bridge doesn't
  simply receive a trajectory and go quiet — it must report progress and a
  final success/failure result back, so MoveIt can decide whether to continue
  or abort a sequence.
- `move_group` also owns the **world model** (the planning scene, Section 2).
  It's a calculator *with memory of the environment* — which is what makes
  milestone M3 meaningful.

### The gap in our setup

MoveIt delivers trajectories over a ROS2 **action** named
`FollowJointTrajectory`. On a real UR5e, the `ros2_control` framework provides
that action. Isaac Sim does not.

What Isaac Sim gives us instead (per `SETUP_NOTES.md`) is a plain **topic**,
`/joint_command`: publish a `sensor_msgs/JointState` and the Action Graph's
Articulation Controller applies those angles.

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

Filling that hole is the job of `trajectory_bridge.py` (Section 5). It is the
only genuinely new *logic* in this project — everything else is configuration.

---

## 2. Vocabulary you need

**URDF** — *Unified Robot Description Format.* An XML file describing the
robot's physical structure: links (rigid bodies), joints (how links connect and
move), joint limits, visual meshes, and collision meshes. Ours is
`src/ur5e_isaac_control/urdf/ur5e.urdf`.

URDF describes *physics and geometry*. It says nothing about intent.

**SRDF** — *Semantic Robot Description Format.* Adds the meaning MoveIt needs
on top of the URDF:

- **Planning groups**: which joints form a controllable unit. Ours is
  `ur_manipulator` — the chain from `base_link` to `tool0` (the flange).
  When you ask MoveIt to plan, you always name a group.
- **Named states**: human-friendly poses, e.g. `home`, `up`.
- **Disabled collision pairs**: pairs of links that are physically adjacent and
  therefore *always* "touching". Without this, the collision checker would
  report the robot as permanently in collision with itself and refuse to plan.

**Planning scene** — MoveIt's model of the world: the robot's current state
plus any obstacles you've registered (conveyor, table, the cube). Collision
checking happens against this. Right now ours contains only the robot; adding
the conveyor is milestone M3.

### The three-way split (a very easy thing to get wrong)

A natural but incorrect assumption is that the SRDF, being the "semantic" file,
is where the robot learns about its environment — the table, the conveyor, tags
it should detect. **It is not.** The SRDF never mentions the outside world. It
is *entirely about the robot itself*.

If the URDF is the robot's **anatomy**, the SRDF is **annotations on that
anatomy**: "these 6 joints together are a thing called `ur_manipulator`", "this
pose is called `home`", "`shoulder_link` and `upper_arm_link` are bolted
together, stop reporting them as colliding".

The environment lives somewhere else entirely — a third thing, which is
*runtime state* rather than a file:

| | What it holds | When it's set |
|---|---|---|
| **URDF** | The robot's body: links, joints, limits, meshes | Authored once, static file |
| **SRDF** | The robot's self-knowledge: planning groups, named poses, self-collision pairs | Authored once, static file |
| **Planning scene** | **The world: table, conveyor, the cube being picked** | **Runtime, in `move_group`'s memory — changes as things move** |

Two consequences that shape the remaining milestones:

- **M3 (teaching MoveIt the conveyor exists) is planning-scene code, not SRDF
  editing.** You populate the scene at runtime by publishing collision objects.
- **M1b (adding the gripper) is a URDF change**, because the gripper is part of
  the robot's *body*, not part of the world.

And the AprilTag pipeline (`apriltag_pose.py`) is **perception** — it produces a
*pose*. That pose is then inserted into the planning scene as an obstacle to
avoid or used as a grasp target. It never touches the SRDF.

**tool0** — ROS-Industrial's standard name for the bare flange frame at the end
of a UR arm. Note this is the *flange*, not the gripper fingertips — which
matters, and is why milestone M1b adds a `grasp_tcp` frame.

**Action vs Topic** — a topic is fire-and-forget broadcast. An action is a
long-running request with progress feedback, a final result, and the ability to
cancel. Executing a trajectory takes seconds and can fail, so it's an action.

**move_group** — the central MoveIt node. It loads the URDF + SRDF + config,
holds the planning scene, answers planning requests, and dispatches execution.
"Running MoveIt" essentially means "running `move_group`".

---

## 3. Why we could reuse UR's config (and where it fell short)

Universal Robots ships an official MoveIt configuration package,
`ur_moveit_config`, and it was **already installed** on this system. Because
our URDF uses the exact standard UR joint and link names, that config applies
to us directly — a large amount of expert-tuned setup we don't have to write.

Here is every file in it that we actually consume, and what each one is for:

| File | Purpose |
|---|---|
| `srdf/ur.srdf.xacro` | The planning group `ur_manipulator`, named states, and the self-collision exclusions described above. |
| `config/kinematics.yaml` | Which IK solver to use for the group. Ours: `KDLKinematicsPlugin`, a general numerical solver. |
| `config/joint_limits.yaml` | Acceleration limits (5.0 rad/s² per joint). URDF carries velocity and effort limits but *not* acceleration, and MoveIt needs acceleration to compute timing. |
| `config/ompl_planning.yaml` | Selects the OMPL planning library and the pre/post-processing adapters (validate workspace, check start state, add time parameterisation). |
| `config/moveit_controllers.yaml` | **The link to our bridge** — see below. |
| `config/moveit.rviz` | A ready-made RViz layout with the Motion Planning panel already set up. |

### The one file that dictates our bridge's design

`moveit_controllers.yaml` is how MoveIt learns *where to send trajectories*. It
declares a controller named `scaled_joint_trajectory_controller`, marked
`default: true`, of type `FollowJointTrajectory`, with action namespace
`follow_joint_trajectory`, covering exactly our six joints.

ROS2 composes those into a full action name:

```
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

Our bridge must advertise **exactly** that name, or MoveIt won't find it. (We
could instead override the YAML with our own name — but matching costs nothing,
so we match.)

### Why we could NOT use UR's launch file

`ur_moveit_config` also ships `ur_moveit.launch.py`, and the obvious move would
be to just run it. Reading it shows two independent reasons that fails here:

1. It launches a node from the package `ur_robot_driver`, and gates `move_group`
   and RViz behind that node exiting. **`ur_robot_driver` is not installed** —
   so nothing ever starts. It's the driver for talking to *physical* UR hardware
   over the network, which we deliberately don't want in the loop.
2. More fundamentally, its config builder loads **only the SRDF — never a
   URDF**. It assumes the real-robot driver already published the robot
   description. We have no such driver.

So we write our own launch file that keeps 100% of the config reuse and drops
only the hardware-driver plumbing.

---

## 4. What `launch/moveit.launch.py` does

A ROS2 launch file is a Python script that starts a set of nodes at once with
the right parameters. Nothing MoveIt-specific about the concept — without it
you'd open four terminals and start four programs by hand, each needing a pile
of configuration.

Ours starts four nodes:

| Node | Job | What breaks without it |
|---|---|---|
| `robot_state_publisher` | Reads `/joint_states` from Isaac Sim + the URDF, computes where every link *is* in 3D, publishes as TF | RViz can't draw the arm; MoveIt doesn't know the current pose |
| `move_group` | MoveIt itself — planning, scene, dispatch | No planning at all |
| `trajectory_bridge` | Our adapter (Section 5) | Planning works, but **Execute does nothing** |
| `rviz2` | The GUI, with UR's ready-made MoveIt layout | No visuals; planning only via code |

### Planning and executing are separately testable

That third row is the practical shape of this milestone, and it's the main
debugging tool for the live test. Launch without the bridge
(`launch_bridge:=false`) and RViz will still plan and animate a *ghost* arm —
that's MoveIt working correctly. Only the **real** arm moving proves the bridge
works.

So: **Plan succeeds but Execute does nothing → suspect the bridge. Plan itself
fails → suspect the config/URDF/SRDF.**

### Important: Isaac Sim must be running even to *plan*

`move_group` has to know where the arm currently *is* before it can plan a path
from there. That comes from `/joint_states` -> `robot_state_publisher` -> TF.

Earlier in this project a `joint_state_publisher` node stood in as a fake source
so planning could be tested standalone. It was removed (it never worked under
the DDS profile — see Section 6), which means **Isaac Sim is now the only source
of joint states.** With it not running, RViz will fail to fetch the current
robot state and planning fails before it even starts.

The plan-vs-execute split above still isolates *which half of MoveIt* is broken
— but both halves require Isaac Sim to be live.

The interesting part is how the configuration is assembled, via
`MoveItConfigsBuilder`:

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

Read that as: *"take our URDF, and pair it with all of UR's semantic and
planning configuration."* Relative paths resolve inside `ur_moveit_config`;
our URDF is passed as an absolute path since it lives in our own package.

**One subtlety worth knowing:** the SRDF's `<robot name="...">` must match the
URDF's `<robot name="...">`, or MoveIt rejects the pair. UR's SRDF takes the
name as an argument, and our URDF declares `name="ur5e"` — hence
`{'name': 'ur5e'}`.

**A harmless warning you will see:** `Cannot infer URDF from
.../ur_moveit_config -- using config/ur5e.urdf`. That's the builder's
constructor guessing before we override it on the next line. Both the URDF and
SRDF are explicitly set, so the guess is discarded. Ignore it.

**Another harmless error:** `No 3D sensor plugin(s) defined for octomap
updates`. MoveIt can build a voxel obstacle map from a depth camera; we haven't
configured one. Only relevant if we later do camera-based obstacle avoidance.

---

## 5. What `trajectory_bridge.py` does

This is the adapter that fills the gap from Section 1.

### a) The action name is a handshake

MoveIt does not discover controllers by looking around. It reads
`moveit_controllers.yaml`, sees a controller named
`scaled_joint_trajectory_controller`, and looks for an action server at exactly:

```
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

Our bridge advertises precisely that string. If it were off by one character,
MoveIt would plan fine and then fail on Execute with *"Unable to identify any
set of controllers"*. Worth recognising that message — it means a **name
mismatch**, not broken execution logic.

### b) What a trajectory actually contains

A planned 3-second motion arrives as roughly 30 points:

```
point[0]:  positions=[0.00, -1.57,  0.00, -1.57,  0.00, 0.00]   time_from_start=0.00s
point[1]:  positions=[0.04, -1.55,  0.03, -1.56, -0.01, 0.00]   time_from_start=0.11s
point[2]:  positions=[0.09, -1.52,  0.07, -1.55, -0.03, 0.00]   time_from_start=0.23s
...
point[29]: positions=[0.80, -1.00,  1.00, -1.50, -1.57, 0.00]   time_from_start=3.00s
```

Note the timestamps are **not evenly spaced**, and the motion starts slow,
speeds up, then slows down. That is MoveIt respecting acceleration limits: it
planned a *velocity profile*, not merely a path.

### c) The core trick: sample by elapsed time

Isaac Sim wants a smooth stream of "go here now". So 60 times a second the
bridge asks *how long since I started?*, finds the two waypoints bracketing that
instant, and linearly interpolates between them.

At t = 0.17 s — between point[1] (0.11 s) and point[2] (0.23 s), 50% through
that segment — it publishes the midpoint of those two poses.

**Why not just replay the 30 points in order?** That would discard the timing.
MoveIt carefully worked out *when* each point should be reached; replaying at
our own fixed rate produces a different, possibly jerky or limit-violating
motion. Sampling against the wall clock reproduces MoveIt's intended profile.

Steps in code: reject empty trajectories; loop at 60 Hz sampling and publishing
to `/joint_command` as a `JointState`; publish action feedback each tick
containing desired position, actual position (from `/joint_states`), and the
error between them.

### d) One honest simplification

Each trajectory point also carries **velocities and accelerations**, and our
bridge **ignores them** — it sends positions only.

That is the correct call here, because the Action Graph wires `/joint_command`
into the Articulation Controller's *Position Command* input (see
`SETUP_NOTES.md`); Isaac Sim's PD drives take position targets. But it is a real
simplification: we drop velocity feedforward, which on real hardware improves
tracking. **If motion looks laggy under fast conveyor picks (M5), this is the
first thing to revisit.**

### e) Why the goal check uses a tolerance

Because those PD drives *track* a target with a small lag rather than snapping
to it, the arm never lands exactly on the final commanded angle. After the
trajectory's duration elapses the bridge pauses briefly, then compares actual
joint positions against the final target with a tolerance (default 0.05 rad,
about 2.9 degrees). Demanding exactness would make every goal report failure
despite the arm having moved correctly.

The node also carries forward two precautions proven in
`joint_command_publisher.py` and documented in `SETUP_NOTES.md`: warn if nothing
is subscribed to `/joint_command` (Isaac Sim not playing → commands silently
vanish), and read current joint positions from `/joint_states` rather than
assuming a starting pose.

### Testing the maths without a robot

The interpolation lives in a plain function, `sample_trajectory(points, t)`,
deliberately kept free of ROS types so it can be tested directly. Verified
behaviour: correct midpoint interpolation, clamping before the start and after
the end, and no division-by-zero on a zero-duration segment.

---

## 6. What has been verified, and what has not

**Verified on this machine:**

- MoveIt core, `ur_moveit_config`, `moveit_servo`, RViz plugin, and the
  `control_msgs` action types are all installed.
- Our URDF pairs cleanly with UR's SRDF; the group, IK solver, and controller
  list resolve correctly in the assembled config.
- `move_group` starts and reports **"You can start planning now!"**.
- The bridge advertises `/scaled_joint_trajectory_controller/follow_joint_trajectory`
  and it appears in `ros2 action list`.
- Trajectory interpolation maths, including edge cases.

**Not yet verified — the actual point of the exercise:**

- MoveIt planning a motion and the **real Isaac Sim arm executing it**. This
  needs Isaac Sim loaded and playing, and is the next checkpoint.

**Known environment quirk (worked around, not fixed):** the Fast-DDS unicast
loopback profile that makes Isaac Sim ↔ WSL2 work also breaks *latched*
(`TRANSIENT_LOCAL`) topic delivery between two nodes inside WSL2. This surfaced
as `joint_state_publisher` never receiving `/robot_description`. It does **not**
affect us, because our launch file passes the URDF and SRDF to `move_group` and
RViz as **parameters** rather than over a topic, and because ordinary action
traffic was confirmed working under the profile. We removed the
`joint_state_publisher` scaffolding rather than chase the DDS issue — Isaac Sim
is the real source of `/joint_states` anyway.

---

## 7. NEXT SESSION: the live test

Everything above is built and unit-tested. The end-to-end test — MoveIt
planning a motion and the real Isaac Sim arm executing it — has **not been run
yet**. That is the next step.

**Prerequisites, in order. Do not skip the check at step 2.**

1. **Isaac Sim (Windows):** open the scene, press **Play**. If the stage or
   Action Graph was edited since the last Play, do a clean **Stop -> Play**
   cycle first (the simulation-view invalidation gotcha in `SETUP_NOTES.md`).

2. **WSL2 — verify the bridge from Windows is alive before involving MoveIt:**
   ```bash
   ros2 topic echo /joint_states
   ```
   Data should stream. If nothing appears, stop — the problem is the Isaac Sim
   ROS2 bridge, not MoveIt. Re-check `SETUP_NOTES.md`'s restart checklist.

3. **Launch MoveIt:**
   ```bash
   cd ~/isaac_sim_ros
   source install/setup.bash
   ros2 launch ur5e_isaac_control moveit.launch.py
   ```
   RViz opens with the Motion Planning panel.

4. **Plan:** drag the interactive marker at the arm's tip to a new pose, click
   **Plan**. A semi-transparent *ghost* arm should animate the proposed path.

5. **Execute:** click **Execute**. The **real Isaac Sim arm** should move.

**Interpreting the outcome:**

| Result | Where the problem is |
|---|---|
| Step 4 fails ("failed to fetch current robot state") | Isaac Sim not playing, or `/joint_states` not arriving |
| Step 4 fails (no plan found) | Config, URDF/SRDF, or the goal pose is unreachable |
| Step 4 works, step 5 does nothing | The bridge — check its terminal output |
| Step 5 errors "unable to identify any set of controllers" | Action name mismatch (Section 5a) |
| Both work | **M1 + M2 complete.** Next: M0 (gripper) and M3 (planning scene). |

The RViz Motion Planning panel is dense and not obvious on first use — worth
walking through its tabs before clicking, rather than hunting for the buttons.

## 8. Concepts still ahead

Deliberately not covered yet; each arrives with its milestone.

- **Collision objects / planning scene** (M3) — teaching MoveIt that the
  conveyor and table exist.
- **TCP and grasp frames** (M1b) — planning to the fingertips instead of the
  bare flange.
- **Cartesian vs joint-space goals** — "move in a straight line" vs "get there
  however you like".
- **Predict-and-intercept** (M5) — why picking a *moving* object is not solved
  by re-planning continuously, and what is done instead.
- **MoveIt Servo** (M6) — continuous velocity streaming for real-time tracking,
  bypassing plan-then-execute entirely.
