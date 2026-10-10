# Vision-Guided AprilTag Sorting with a UR5e in Isaac Sim

A learning project: a simulated UR5e with a Robotiq 2F-85 gripper picks AprilTag-marked cubes off a
conveyor in **NVIDIA Isaac Sim**, driven over **ROS 2 Jazzy**, with **MoveIt 2 + NVIDIA cuMotion**
for motion planning.

**Goal:** cubes spawn on a running conveyor at an interval. The camera reads each cube's AprilTag
as **Good** or **Bad**. The robot picks Good cubes *off the moving belt* into the collection bin;
Bad cubes ride off the belt's end into the waste bin.

## Current state (2026-10-10)

| Stage | Status |
|---|---|
| Isaac ↔ ROS 2 bridge, sim-time alignment, gripper control | ✅ verified |
| Perception: AprilTag → cube pose in the robot frame (~0.3 mm error) | ✅ verified |
| Fixed-position pick → bin, planned with MoveIt's Pilz planner | ✅ verified (2026-09-27) |
| **Fixed-position pick → bin, planned with cuMotion** | ✅ working; one known defect (below) |
| Straight-line vertical moves for the cuMotion path | ⬜ next |
| Bins in their final places + static collision scene | ⬜ |
| Picking from the moving belt | ⬜ |
| Timed spawning + Good/Bad sorting by tag ID | ⬜ |

Measured on the cuMotion pick: no IK branch flips, worst joint tracking error 0.046 rad,
fingertips within ~1 mm of the target at the grasp. cuMotion uses ~0.1 GB of GPU memory.

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

- **Isaac Sim** (native) simulates the robot, conveyor and camera. Action Graphs publish
  `/isaac_joint_states`, `/clock`, `/rgb` and `/camera_info`, and apply `/isaac_joint_commands`.
- **Host ROS 2 nodes** (`src/ur5e_isaac_control`):
  - `cube_pose_publisher` detects the 36h11 tag with OpenCV and `solvePnP`, then corrects depth by
    intersecting the camera ray with the known tag plane.
  - `pick_orchestrator` runs the sequence home → approach → descend → grasp → lift → bin →
    release → home. For each move it gets IK from MoveIt's `/compute_ik`, seeded at the current
    pose, and sends a joint goal to `move_group`.
- **Isaac ROS 4.6 container** (Docker, `--network host`): NVIDIA's `ur_isaac_sim.launch.py`. It
  runs `move_group` with cuMotion as the planner, plus ros2_control's `joint_trajectory_controller`.
  The `topic_based_ros2_control` plugin connects that controller to Isaac's topics.
- Everything runs on **sim time** (`/clock`). The sim runs at ~0.4× real time on an RTX 4060
  Laptop GPU (8 GB).

## Repository layout

| Path | Contents |
|---|---|
| `src/ur5e_isaac_control/` | ROS 2 package: perception, orchestrator, launch files, URDF/SRDF for the Pilz path |
| `src/ur5e_gripper_moveit_config/` | MoveIt Setup Assistant output (SRDF source for the Pilz path) |
| `isaac_ros/` | Container side: `Dockerfile.cumotion`, Isaac ROS CLI config, patched launch file. Same layout as `~/workspaces/isaac_ros-dev/` |
| `scenes/UR5e_Sim_V1.usd` | The Isaac stage (robot, conveyor and bins referenced from NVIDIA's online assets) |
| `scripts/` | Helpers run inside Isaac's Script Editor: camera setup and extrinsic, scene measurement, tag decal, cube spawner |
| `apriltags/` | 36h11 tags, ids 0–3 |

## Prerequisites

| | Requirement | Developed on |
|---|---|---|
| OS | Ubuntu 24.04 (native; WSL2 caused DDS discovery problems) | 24.04 |
| GPU | NVIDIA, Ampere or newer, **≥ 8 GB VRAM** (Isaac Sim alone uses ~5 GB) | RTX 4060 Laptop, 8 GB |
| Driver / CUDA | Driver ≥ 595, CUDA ≥ 13.2 (required by Isaac ROS 4.6) | 595.91 / 13.2 |
| Disk | ~40 GB free (Isaac ROS image + build) | — |
| Isaac Sim | 6.0.1, installed at `~/isaacsim` | 6.0.1 |
| ROS 2 | Jazzy at `/opt/ros/jazzy` | Jazzy |
| Docker | Docker Engine + NVIDIA Container Toolkit, user in the `docker` group | Docker 29.9 |
| Internet | Needed when opening the stage: robot, conveyor, bins and warehouse load from NVIDIA's online asset library (Isaac 6.0 paths) | — |

### One-time setup

**1. Host ROS workspace**

```bash
cd ~ && git clone https://github.com/aravinds1610/isaacsim_ros_ws.git && cd isaacsim_ros_ws
rosdep install --from-paths src --ignore-src -y --skip-keys warehouse_ros_mongo   # not on Jazzy; unused
pip install --user --break-system-packages --no-deps ikpy   # kinematics.py; no rosdep key
colcon build --symlink-install
echo 'source ~/isaacsim_ros_ws/install/setup.bash' >> ~/.bashrc
```

**2. Isaac ROS CLI** (NVIDIA's apt source, pinned to release 4.6. Release 5.0 targets ROS 2
Lyrical and will not work with Jazzy):

```bash
k="/usr/share/keyrings/nvidia-isaac-ros.gpg"
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor | sudo tee -a $k > /dev/null
echo "deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-4.6 noble main" \
  | sudo tee /etc/apt/sources.list.d/nvidia-isaac-ros.list
sudo apt-get update && sudo apt-get install isaac-ros-cli
sudo isaac-ros init docker
```

**3. Isaac ROS workspace and container image** (~10 min to build):

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src ~/.aws   # ~/.aws: the CLI mounts it; avoids a root-owned dir
echo 'export ISAAC_ROS_WS="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev/}"' >> ~/.bashrc
source ~/.bashrc
cp -r ~/isaacsim_ros_ws/isaac_ros/. ~/workspaces/isaac_ros-dev/
isaac-ros activate --build-local   # builds, then drops you into the container; `exit` to leave
```

After editing anything in `isaac_ros/`, copy it over again. Rerun `--build-local` only if the
Dockerfile changed.

The stage finds the AprilTag texture by a path relative to itself
(`../apriltags/…`), so the clone can live anywhere. Keep `scenes/` and `apriltags/` side by side.

## Running a pick

Each run: open `scenes/UR5e_Sim_V1.usd` in Isaac, press Play, cube in view.

```bash
# 1. container
isaac-ros activate
ros2 launch /workspaces/isaac_ros-dev/launch/ur_isaac_sim.launch.py ur_type:=ur5e
# 2. host
ros2 launch ur5e_isaac_control pick_cumotion.launch.py
# 3. host — fire one pick
ros2 service call /pick_orchestrator/pick_once std_srvs/srv/Trigger
```

To record a run for analysis:
`ros2 bag record -o ~/bags/<name> /clock /isaac_joint_commands /isaac_joint_states /cube_pose`.

## Known issues

- **cuMotion's "straight" moves curve.** cuMotion plans in joint space and has no straight-line
  mode. During the final descent, the fingertips wander up to ~10 mm sideways. Each finger has
  ~17.5 mm of clearance around the cube, so this likely caused one pushed-and-rotated cube.
  Planned fix: `/compute_cartesian_path` + `/execute_trajectory` for the vertical moves.
- **The bins are swapped in the stage.** The robot currently drops into `Sorting_Bin_1` (meant
  for waste). `RecollectionBin` (collection) sits on the floor, about 2.7 m away and out of reach.
- **Stop/Play in Isaac resets sim time to 0.** Restart both ROS launches afterwards; the
  controllers and `move_group` otherwise keep stale timing and refuse new trajectories. To reset
  a cube, drag it while the sim plays instead.
- **No gripper in the cuMotion collision model.** NVIDIA's model is the bare arm, so the
  orchestrator aims `tool0` and adds a 0.11181 m fingertip offset. The planning scene is empty, so
  the arm may plan through the conveyor or bin (harmless in sim).
- **Two defects in NVIDIA's Isaac ROS 4.6 release, patched in `isaac_ros/`:**
  - The prebuilt `topic_based_ros2_control` crashes against Jazzy's ros2_control 4.48.1. It was
    compiled against an older `hardware_interface`, a binary (ABI) mismatch. The Dockerfile
    rebuilds it from upstream source.
  - `ur_isaac_sim.launch.py` hard-codes `ur10e.xrdf`, so on a UR5e most start poses were
    rejected as colliding. The patched copy uses the XRDF matching `ur_type`.
- **Pose goals sent straight to cuMotion can flip the elbow**, because its own IK picks any
  solution branch. Hence the orchestrator's IK-then-joint-goal step (`ik_then_joint_goal`).
- **AprilTag detection is near its limit:** the tag is ~32 px across (4 px per cell) at the
  current camera height. That's fine for a stationary cube; a moving one may need a closer camera
  or a bigger tag.
- **`wrist_1` lag on the old Pilz path.** On that path (`pick_demo.launch.py`), `wrist_1` gets
  pushed ~1 rad off target during large moves, then snaps back. This hasn't been seen on the
  cuMotion path, and the cause was never isolated. The Pilz path also needs the stage's joint
  topics renamed back to `/joint_states` / `/joint_command`.
- **GPU memory is the hardware limit:** 6.2 of 8 GB in use, GPU at 100%. Measure any new GPU
  consumer (depth camera, nvblox, …) before keeping it.

## References

- [Isaac ROS cuMotion (release 4.6)](https://nvidia-isaac-ros.github.io/v/release-4.6/repositories_and_packages/isaac_ros_cumotion/index.html) ·
  [cuMotion MoveIt plugin](https://nvidia-isaac-ros.github.io/v/release-4.6/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion_moveit/index.html) ·
  [Isaac ROS getting started](https://nvidia-isaac-ros.github.io/v/release-4.0/getting_started/index.html) ·
  [Isaac ROS releases](https://nvidia-isaac-ros.github.io/releases/index.html)
- [isaac_ros_cumotion source (XRDF files)](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_cumotion/tree/release-4.6)
- [topic_based_ros2_control (PickNik)](https://github.com/PickNikRobotics/topic_based_ros2_control)
- [MoveIt 2](https://moveit.picknik.ai/main/index.html) · [ros2_control](https://control.ros.org/jazzy/index.html)
- [Isaac Sim documentation](https://docs.isaacsim.omniverse.nvidia.com/)
- [Universal Robots ROS 2 description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) ·
  [AprilTag](https://april.eecs.umich.edu/software/apriltag)
