# Isaac Sim + ROS2 MoveIt — Migration Plan

**Purpose:** I'm moving off the *Isaac Sim on Windows + ROS2 in WSL2* setup, which
NVIDIA has now **deprecated** and which breaks MoveIt. This document is
self-contained so the work can continue in a fresh conversation (e.g. on
claude.ai) away from my dev machine.

**Two plans:** **Plan A (Pixi, native Windows) — recommended, try first.**
**Plan B (dual-boot Ubuntu) — fallback, higher cost and risk.**

Written 2026-08-08. Supersedes the earlier dual-boot-only draft.

---

## 0. How I'd like you to work with me

I'm learning robotics/MoveIt, not just requesting a working artifact:

- **Concept before commands.** Explain what a step does and why, before I run it.
- **One step at a time.** Stop and check I've understood before moving on.
- **Be explicit about what's verified vs. assumed.** I'd rather hear "I haven't
  confirmed this" than a confident guess.
- **Correct me plainly** if I state something wrong.

I have deep hands-on experience with the Isaac Sim side; I started MoveIt at zero.

---

## 1. Project context

**Goal:** a conveyor-based picking system — a UR5e arm with a Robotiq 2F-85
gripper picks AprilTag-marked cubes off a moving conveyor in Isaac Sim, planned
and executed via **MoveIt2**. Target is a *dynamic tracking pick* (intercept the
moving item), starting from a known/fixed object pose before adding perception.

**Workspace:** `~/isaac_sim_ros` (WSL2), package `ur5e_isaac_control`
(ament_python), ROS2 **Jazzy**.

| File | What it is |
|---|---|
| `docs/SETUP_NOTES.md` | Windows↔WSL2 bridge build notes. The **Action Graph section stays valid**; the DDS sections become obsolete. |
| `docs/MOVEIT_PROJECT_PLAN.md` | Milestones M0–M6, architecture |
| `docs/MOVEIT_EXPLAINED.md` | Teaching notes on MoveIt concepts, the launch file, the bridge |
| `launch/moveit.launch.py` | Brings up `robot_state_publisher`, `trajectory_bridge`, `move_group`, `rviz2` |
| `ur5e_isaac_control/trajectory_bridge.py` | Custom `FollowJointTrajectory` action server → `/joint_command` |
| `ur5e_isaac_control/joint_command_publisher.py` | Simple joint streamer (proven working) |
| `ur5e_isaac_control/joint_controller_gui.py` | Tk slider panel for jogging joints |
| `ur5e_isaac_control/kinematics.py` | ikpy-based FK/IK (independent of MoveIt) |
| `ur5e_isaac_control/apriltag_pose.py` | OpenCV AprilTag pose estimation (later) |
| `urdf/ur5e.urdf` | Robot description (robot name `ur5e`) |

**MoveIt config approach:** reuse official `ur_moveit_config` SRDF / kinematics /
joint limits / OMPL / controller YAML, but supply *our own* URDF. We deliberately
do **not** use `ur_moveit.launch.py` — it depends on `ur_robot_driver` (physical
hardware) and never loads a URDF.

**Controller action name** MoveIt expects, which `trajectory_bridge.py` advertises:

```
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

### Milestones

- **M0** — Gripper ROS2 wiring (2F-85 controllable in Isaac Sim; no ROS2 code yet)
- **M1** — MoveIt bring-up ✅ *built, planning verified*
- **M1b** — URDF gripper geometry + `grasp_tcp` frame
- **M2** — `FollowJointTrajectory` bridge ✅ *built, execution NOT verified*
- **M3** — Planning-scene collision objects (conveyor, table)
- **M4** — Fixed-point pick-and-place
- **M5** — Dynamic intercept (predict-and-intercept)
- **M6** — *(stretch)* closed-loop tracking via `moveit_servo` + AprilTag

---

## 2. Why migrate — the root cause

Isaac Sim runs on **Windows**; ROS2 runs in **WSL2**. WSL2 mirrored networking
breaks UDP multicast, so DDS discovery fails across the boundary. The workaround
was a Fast-DDS XML profile (`~/.ros/fastdds_unicast_profile.xml`) forcing traffic
over unicast loopback.

**That profile works for topics but breaks MoveIt.** Verified by A/B test:

| Configuration | Result |
|---|---|
| Profile active | `move_group` plans successfully, but the result **never reaches RViz** — panel hangs on "Planning…"; `get_planning_scene` also times out |
| Profile removed (WSL2-local) | Plan completes in **0.017 s**, preview animates, all fine |

**Topics work; request/reply (services, and the action result fetch) does not.**

Suspected cause: the profile pins every participant to one
`defaultUnicastLocatorList` locator with no port, so with 5+ nodes the
participants collide instead of each getting a distinct port. Consistent with a
2-process setup always working and only the multi-node MoveIt stack failing.

### Already tried and ruled out

1. `useBuiltinTransports` → `true` — no effect (reverted).
2. Stripping the profile from RViz only — TF still crossed, RPC still failed, so
   the problem follows `move_group`, not the requester.
3. **Fast-DDS Discovery Server** on `127.0.0.1:11811` — dead end. A raw UDP
   listener proved Isaac Sim *does* honour `ROS_DISCOVERY_SERVER` and its packets
   *do* cross (`DS_EPROSIMA` registrations arrived), and the server worked for
   WSL2-local nodes — but Isaac Sim's topics never appeared. Likely a Fast-DDS
   version mismatch between Isaac Sim's bundled DDS and Jazzy's.
4. **Isaac Sim inside WSL2** — impossible on this machine. No NVIDIA Vulkan in
   WSL2: no `nvidia_icd.json` and **no Vulkan library to point an ICD at**
   (`/usr/lib/wsl/lib` has CUDA, OptiX, NVENC, DXCore — nothing Vulkan). Known
   NVIDIA regression from ~581.xx; this machine is on 610.88. Isaac Sim's RTX
   renderer requires hardware Vulkan.

### The decisive discovery

NVIDIA now marks **WSL2 as "(Deprecated)"** and recommends **native Windows ROS 2
Jazzy via Pixi**, which uses **Zenoh** (`rmw_zenoh_cpp`) rather than Fast-DDS.
That removes the host boundary entirely *and* removes the middleware that failed.

Docs: <https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_ros_other_platforms.html>

---

# PLAN A — Pixi native Windows *(recommended)*

## A1. Why this first

| | Plan A (Pixi) | Plan B (dual-boot) |
|---|---|---|
| Time | ~30–60 min | several hours |
| Risk to data | **none** — it's a folder | decrypt `F:`, repartition, BitLocker lockout risk |
| Reversible | delete the folder | not really |
| Keeps Isaac Sim + USD scene | yes, untouched | reinstall everything |
| Vendor-supported | **yes, current** | yes |

Trying A costs almost nothing and doesn't make B harder later.

## A2. Verified package availability (checked on anaconda.org, RoboStack jazzy)

| Package | Version | win-64 |
|---|---|---|
| `ros-jazzy-moveit` | 2.12.4 | ✅ |
| `ros-jazzy-ur-moveit-config` | **3.8.0** | ✅ (the exact version in use) |
| `ros-jazzy-ur-description` | 3.5.1 | ✅ |
| `ros-jazzy-moveit-servo` | 2.12.4 | ✅ |

`ros-jazzy-moveit >=2.12.4,<3` is **already a dependency in NVIDIA's own
`jazzy_ws/pixi.toml`**, which also pins Isaac Sim **6.0.1.0** — matching the
installed version.

## A3. Setup steps

```powershell
# 1. Install pixi (verify the current command at https://pixi.sh)
powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"

# 2. Clone NVIDIA's workspace
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git C:\IsaacSim-ros_workspaces
cd C:\IsaacSim-ros_workspaces\jazzy_ws

# 3. Resolve the environment (downloads ROS2 Jazzy, MoveIt, compilers, colcon)
pixi install

# 4. Add the UR packages (MoveIt is already included)
pixi add ros-jazzy-ur-moveit-config ros-jazzy-ur-description

# 5. Build
pixi run build

# 6. Launch Isaac Sim with the ROS2 bridge pre-enabled
pixi run sim
```

`pixi.toml` sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` on environment activation, and
Pixi auto-sources `install\setup.bat` inside `pixi shell` / `pixi run`. No manual
sourcing.

Channels used: `https://prefix.dev/robostack-jazzy` and
`https://prefix.dev/conda-forge`. Tasks available: `build`, `test`, `clean`,
`sim`, `headless`.

## A4. Moving `ur5e_isaac_control` across

From WSL2 (WSL can write to `C:` via `/mnt/c`):

```bash
cp -r ~/isaac_sim_ros/src/ur5e_isaac_control \
      /mnt/c/IsaacSim-ros_workspaces/jazzy_ws/src/
```

Then `pixi run build` again.

### Code changes needed

1. **`launch/moveit.launch.py` — remove the `rviz_env` block.** It strips
   `FASTRTPS_DEFAULT_PROFILES_FILE` from the RViz node; that was a WSL2-only
   workaround and is meaningless under Zenoh. Delete `rviz_env = ...`,
   `rviz_env.pop(...)`, and the `env=rviz_env,` argument.
2. **Check extra Python deps** are present in the pixi env: `ikpy`
   (`kinematics.py`), `opencv` (`apriltag_pose.py`), `tkinter`
   (`joint_controller_gui.py`). Add via `pixi add` as needed — these are *not*
   needed for the core MoveIt path, so don't let them block progress.
3. **Nothing else should need changing.** `trajectory_bridge.py` and
   `joint_command_publisher.py` are pure `rclpy` with no Linux-specific calls.

### Windows env cleanup

Remove the old Fast-DDS variable — it's ignored by Zenoh but it also affects
other Windows apps (notably **MATLAB ROS Toolbox**, per `SETUP_NOTES.md`):

```cmd
reg delete "HKCU\Environment" /v FASTRTPS_DEFAULT_PROFILES_FILE /f
```

Also drop any `netsh portproxy` rules that were added for DDS.

## A5. What to verify, in order

1. `pixi run ros2 topic list` → shows `/joint_states` and `/joint_command`
   *(Isaac Sim must be launched via `pixi run sim` and be **playing**)*
2. `pixi run ros2 topic echo /joint_states --once` → real joint values
3. `pixi run ros2 launch ur5e_isaac_control moveit.launch.py`
4. RViz shows the arm in Isaac Sim's actual pose
5. Drag the interactive marker → **Plan** → preview animates
6. **Execute** → the Isaac Sim arm moves ← *this has never worked yet*

## A6. Known unknowns for Plan A

Be honest about these — they're unverified:

- **Zenoh may need a router daemon** (`ros2 run rmw_zenoh_cpp rmw_zenohd`)
  running before nodes start. Check NVIDIA's docs; this is a common `rmw_zenoh`
  requirement and a likely first stumbling block.
- **MoveIt on Windows is far less trodden than on Linux.** The packages build,
  but edge cases are plausible.
- **RViz2 rendering on Windows** — expected fine, unconfirmed.
- **`pixi run sim` may install its own Isaac Sim copy** (the pixi.toml references
  Isaac Sim 6.0.1.0 as a dependency) rather than using
  `F:\Softwares\IsaacSim-6.0.1`. Budget ~30 GB on `C:` (160 GB free) and check
  whether the existing install can be reused.
- **The USD scene** must be opened in the Isaac Sim that Pixi launches. The
  Action Graph wiring (`SETUP_NOTES.md`) stays valid.

If Plan A fails in a way that looks structural rather than fixable, fall back to
Plan B.

---

# PLAN B — Dual-boot Ubuntu 24.04 *(fallback)*

Only if Plan A fails. Everything below was verified on the machine.

## B1. Hardware inventory (verified)

**System:** Lenovo Legion 5 16IRX9 (model `83DG`)
**OS:** Windows 11 Home Single Language, build **26200**
**CPU:** 32 logical processors
**RAM:** WSL2 got 15.6 GiB at default 50% → ~32 GB physical
**GPU:** RTX 4060 **Laptop**, **8 GB VRAM**, Windows driver 610.88
**Firmware:** UEFI, both disks **GPT**
**Fast Startup:** **ENABLED** — must be turned off
**BitLocker:** ⚠️ **all four volumes encrypted** — see B2

| Disk | Model | Partition | Label | Size | Free |
|---|---|---|---|---|---|
| 0 | SK Hynix (954 GB) | EFI System | — | 0.3 GB | — |
| 0 | | **C:** | Cooper | 501.6 GB | 160.5 GB |
| 0 | | **D:** | Newton | 450 GB | 300.5 GB |
| 0 | | Recovery | WINRE_DRV | 2 GB | 1.2 GB |
| 1 | WD_BLACK SN850X (931.5 GB) | **E:** | Einstein | 465.7 GB | 77 GB |
| 1 | | **F:** | Feynmann | 465.7 GB | **395.4 GB** |

Isaac Sim (Windows) lives at `F:\Softwares\IsaacSim-6.0.1`.

## B2. BitLocker — verified, and it constrains everything

`manage-bde -status` confirms **every volume encrypted**, XTS-AES 128, "Used Space
Only", 100%, **Protection On**, currently unlocked:

| Volume | Role | Key protectors |
|---|---|---|
| **C:** | OS | Numerical Password, **TPM** |
| **D:** | Data | Numerical Password, External Key (auto-unlock) |
| **E:** | Data | Numerical Password, External Key (auto-unlock) |
| **F:** | Data | Numerical Password, External Key (auto-unlock) |

Consequences:

1. **`F:` cannot be shrunk while encrypted.** Must be **fully decrypted**
   (`manage-bde -off F:`) — suspending is *not* enough.
2. **Linux cannot read `D:`, `E:`, `F:` while encrypted.** Any "copy files to NTFS
   for the Linux side" plan is invalid unless that volume is decrypted.
3. **`C:` is TPM-protected with no PIN.** Firmware/boot-order changes extend TPM
   PCRs and can trigger a recovery prompt. The Numerical Password **must** be
   saved off-machine first.

### Step 0 — save every recovery key (before anything)

```powershell
manage-bde -protectors -get C:
manage-bde -protectors -get D:
manage-bde -protectors -get E:
manage-bde -protectors -get F:
```

Store the **Numerical Password** for each off-machine (phone, second device) and
confirm at <https://account.microsoft.com/devices/recoverykey>. Saving them onto
`C:` is useless if `C:` locks you out.

### Decrypting F:

```powershell
manage-bde -off F:              # start
manage-bde -status F:           # poll until "Fully Decrypted"
```

"Used Space Only" with ~70 GB used → far quicker than a full-volume decrypt. AC
power, don't interrupt. **Leave `F:` permanently decrypted** — it becomes the
shared NTFS partition both OSes can use. `D:`/`E:` can stay encrypted (they just
won't be visible from Linux).

### Protecting C: during install

```powershell
manage-bde -protectors -disable C: -RebootCount 0   # stays suspended across reboots
# ...after both OSes boot reliably:
manage-bde -protectors -enable C:
```

## B3. Target layout

Install Ubuntu on **Disk 1**, by shrinking `F:` (after decryption):

- Shrink `F:` 465.7 GB → ~150 GB (keeps its ~70 GB of data)
- ~315 GB for Ubuntu
- **Create a second EFI System Partition on Disk 1** (~1 GB)

**Why a separate ESP:** Windows' bootloader stays on Disk 0, Ubuntu's on Disk 1.
They never touch, Windows updates can't clobber GRUB, and you pick the OS from the
**F12** boot menu. Most dual-boot horror stories come from sharing one ESP.

⚠️ The installer defaults to the *existing* ESP on Disk 0. Use manual ("Something
else") partitioning and explicitly set the bootloader device to **Disk 1**.

| Mount | Size |
|---|---|
| `/boot/efi` | 1 GB (new ESP on Disk 1) |
| `/` | ~300 GB |
| swap | 8–16 GB (optional) |

Budget: Ubuntu ~30 GB, Isaac Sim ~30 GB, Omniverse asset cache 50–100 GB, ROS2 +
MoveIt ~5 GB.

## B4. Pre-flight checklist — IN ORDER

1. **Save all four BitLocker recovery keys off-machine** (B2 Step 0). Most
   important step; everything else is recoverable.
2. **Disable Fast Startup** — Power Options → "Choose what the power buttons do"
   → uncheck *Turn on fast startup*. *(Currently ON; leaving it on makes Linux see
   a dirty NTFS and risks corruption.)*
3. **Fully decrypt `F:`**.
4. **Back up the workspace** — git remote, external USB, or decrypted `F:`.
   `D:`/`E:` will **not** be readable from Linux.
5. **Copy the USD scene** to decrypted `F:` or external media.
6. **Suspend BitLocker on `C:`**.
7. **Check BIOS storage mode** — if the installer can't see the NVMe drives, BIOS
   is in Intel RST/RAID mode; switching to AHCI needs a Windows safe-mode dance
   *first* or Windows won't boot. Check **before** starting.
8. *(Recommended)* Set MUX to **discrete graphics only** (Lenovo Vantage / BIOS).
9. **Shrink `F:` from Windows Disk Management** — only after full decryption.
10. **Full shutdown** — `shutdown /s /t 0`. Not restart, not sleep.

## B5. Do-not list

- ❌ Don't resize partitions with BitLocker active.
- ❌ Don't save recovery keys only on `C:`.
- ❌ Don't interrupt `manage-bde -off F:`; keep on AC power.
- ❌ Don't assume Linux can read `D:`/`E:`.
- ❌ Don't use "Install alongside Windows" / auto-resize. Pre-shrink, install into
  *unallocated space*.
- ❌ Don't use NVIDIA's `.run` driver installer — breaks on kernel updates. Use
  `apt` / "Additional Drivers".
- ❌ **Don't install the `-open` NVIDIA driver variant** — reports of
  open-kernel-module instability on Legion Pro 5 16IRX8 + RTX 40-series laptop
  GPUs, essentially this hardware. Use proprietary `nvidia-driver-###`.
- ❌ Don't change Secure Boot before suspending BitLocker.
- ❌ Don't skip the blue **MOK enrollment** screen after installing the NVIDIA
  driver with Secure Boot on — skip it and the driver silently won't load.
- ❌ Don't delete System / Reserved / Recovery partitions on Disk 0.

## B6. Post-install stack

1. **NVIDIA proprietary driver** — `sudo ubuntu-drivers install`; verify `nvidia-smi`.
2. **Verify Vulkan** (the thing WSL2 couldn't do):
   ```bash
   sudo apt install vulkan-tools && vulkaninfo --summary   # must show NVIDIA, not llvmpipe
   ```
3. **Use Xorg, not Wayland** — pick "Ubuntu on Xorg" via the gear icon at login.
4. **Isaac Sim 6.0.1 for Linux** — ⚠️ install method unverified; check
   <https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/>
5. **ROS2 Jazzy** — `sudo apt install ros-jazzy-desktop`
6. **MoveIt2 + UR config:**
   ```bash
   sudo apt install ros-jazzy-moveit ros-jazzy-ur-moveit-config \
                    ros-jazzy-ur-description ros-jazzy-moveit-servo
   ```
   *(Do **not** install `ur_robot_driver` — physical hardware only.)*

## B7. Post-migration cleanup

Delete all DDS workarounds — they existed only for the WSL2 boundary:

- Remove `export FASTRTPS_DEFAULT_PROFILES_FILE=...` from `~/.bashrc`
- Delete `~/.ros/fastdds_unicast_profile.xml`
- Remove the Windows `setx FASTRTPS_DEFAULT_PROFILES_FILE` value
- Remove `netsh portproxy` rules
- Optionally revert `.wslconfig` `networkingMode=mirrored`
- **Remove the `rviz_env` block from `launch/moveit.launch.py`**

---

# Issues that survive EITHER plan

## Sim-time vs wall-clock

Discovered while debugging, **not fixed**. It's an Isaac Sim configuration issue,
not networking, so it persists under both plans:

```
Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds.
Requested time 1786192283.739997, but latest received state has time 1566.216675.
[WARN] Failed to fetch current robot state.
```

Isaac Sim stamps `/joint_states` with **simulation time** (seconds since Play,
e.g. `1566.2`); ROS2 nodes use **wall-clock epoch** (`1786192283`).
`move_group`'s `current_state_monitor` fails its recency check on the ~56-year gap.

*Soft* failure — planning still works from the last known joint values — but it
should be fixed. Likely fix: publish `/clock` from Isaac Sim's Action Graph and
set `use_sim_time: true` on all nodes. **Needs verifying.**

## Benign warnings (safe to ignore)

- `SEVERE WARNING!!! ... namespace collision ... InteractiveMarkerDisplay` —
  cosmetic RViz2/Jazzy plugin double-registration.
- `Publisher already registered for node name: 'rviz2_moveit'` — duplicate logger
  registration, no functional effect.
- `Action server: /recognize_objects not available` — unused feature.

---

# Where the robotics work resumes

**The end-to-end test that has never succeeded:** Isaac Sim playing → launch the
MoveIt stack → drag the RViz interactive marker → **Plan** → **Execute** → confirm
the real Isaac Sim arm moves.

Genuinely verified so far:

- ✅ MoveIt config loads; `ur5e` model parses; `ur_manipulator` group works
- ✅ OMPL/RRTConnect **plans successfully** (0.017 s on a small motion)
- ✅ `trajectory_bridge` advertises the correct action name and is discoverable
- ✅ `sample_trajectory()` interpolation unit-tested standalone
- ✅ Isaac Sim publishes `/joint_states` with 6 arm joints **and 6 gripper joints**
- ❌ **Execution has never run end-to-end** — MoveIt has never moved the arm

Known simplification in `trajectory_bridge.py`: it sends **positions only**, no
velocity/acceleration feedforward, because Isaac Sim's Articulation Controller
consumes Position Command. First thing to revisit if motion looks laggy during
fast conveyor picks (M5).

Noted for M5: **belt velocity is not defined in
`scripts/isaac_conveyor_spawner.py`** — read it from the USD conveyor prim's PhysX
surface velocity, or derive it empirically from cube motion.

---

# Open questions

1. Does `rmw_zenoh_cpp` require a running `rmw_zenohd` router for this setup?
2. Does `pixi run sim` reuse `F:\Softwares\IsaacSim-6.0.1` or install its own copy?
3. Any known MoveIt-on-Windows pitfalls (RViz, OMPL, plugin loading)?
4. Best way to publish `/clock` from Isaac Sim's Action Graph and wire
   `use_sim_time` through the MoveIt launch file.
5. Should the Robotiq 2F-85 be a second MoveIt planning group, or stay as URDF
   collision geometry plus a separate control path? *(Current plan: the latter —
   `ur_moveit_config`'s SRDF has no gripper group.)*
6. Is 8 GB VRAM sufficient for this conveyor scene? It's at Isaac Sim's minimum.
