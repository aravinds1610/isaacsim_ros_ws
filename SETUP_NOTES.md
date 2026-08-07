# Isaac Sim <-> ROS2 Jazzy Setup Notes (UR5e)

Summary of getting a UR5e in Isaac Sim 6.0.1 (Windows) talking to ROS2 Jazzy
(WSL2) and driving joint commands over the ROS2 bridge. Written up so the
setup is reproducible after a reboot or a fresh WSL2/Isaac Sim install.

## Architecture

- **Isaac Sim 6.0.1** runs natively on **Windows**, not inside WSL2
  (`F:\Softwares\IsaacSim-6.0.1`). It ships its own bundled ROS2 Jazzy
  libraries (`isaacsim.ros2.bridge` / `isaacsim.ros2.core` extensions).
- **ROS2 Jazzy** is installed natively inside **WSL2** (Ubuntu 24.04).
- WSL2 networking mode is **mirrored** (`.wslconfig`:
  `networkingMode=mirrored`), so WSL2 shares Windows' IP addresses and
  loopback interface, rather than getting its own NATed IP.

## The core problem: WSL2 mirrored mode breaks DDS discovery

ROS2's default discovery (Fast-DDS SPDP) relies on **UDP multicast**. This is
a **known, NVIDIA-confirmed bug**: UDP multicast forwarding across the
Windows<->WSL2 boundary is unreliable under mirrored networking mode, even
though ordinary unicast traffic (ping, TCP, plain UDP) passes through fine.
Result: `ros2 topic list` in WSL2 only ever showed the daemon's own default
topics (`/parameter_events`, `/rosout`) — nothing from Isaac Sim, no matter
how correct the OmniGraph, prim paths, or firewall rules were.

NVIDIA's own forums state the *officially supported* path for Windows +
ROS2 is **WSL2 NAT mode + `netsh interface portproxy`** port forwarding, not
mirrored mode. We deliberately avoided that (more disruptive, changes IP
behavior for all of WSL2) and used a documented community workaround instead
(see below), which worked once corrected.

## The fix: unicast-only Fast-DDS profile

A custom Fast-DDS XML profile that disables multicast entirely and forces
**all** DDS traffic — both discovery *and* actual message data — over
loopback (`127.0.0.1`), which mirrored-mode WSL2 does share reliably with
Windows.

File: `~/.ros/fastdds_unicast_profile.xml` (WSL2) — an identical copy also
lives at `C:\Users\aravi\.ros\fastdds_unicast_profile.xml` on Windows for
Isaac Sim to read.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udpv4_transport</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="default_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udpv4_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <defaultUnicastLocatorList>
        <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
      </defaultUnicastLocatorList>
      <builtin>
        <metatrafficUnicastLocatorList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
        </metatrafficUnicastLocatorList>
        <initialPeersList>
          <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
        </initialPeersList>
        <discovery_config>
          <discoveryProtocol>SIMPLE</discoveryProtocol>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
```

Two things about this profile that weren't obvious upfront:

1. **`metatrafficUnicastLocatorList` alone isn't enough.** It only routes
   *discovery* traffic. It's possible for discovery to succeed (topics show
   up in `ros2 topic list`, subscriber counts look right) while actual
   message *data* still silently fails to arrive, because Fast-DDS
   auto-picks a separate locator for user data unless you also set
   `defaultUnicastLocatorList`. This exact asymmetry is what caused
   `/joint_states` (Windows->WSL2) to work before `/joint_command`
   (WSL2->Windows) did.
2. A community-posted version of this XML (NVIDIA forums) included
   `<use_SIMPLE_EndpointDiscoveryProtocol>` and
   `<use_WriterLivelinessProtocol>` tags inside `<discovery_config>` — these
   aren't valid elements in this Fast-DDS XSD version and caused a silent
   XML parse failure (visible via `ros2 topic list --no-daemon` stderr),
   silently falling back to defaults. Removed them; `<discoveryProtocol>`
   alone is sufficient.

### Environment variables required (both sides)

**WSL2** (persisted in `~/.bashrc`):
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # must match Isaac Sim's default
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_unicast_profile.xml
```

**Windows** (set via `setx`, so it survives new terminals — requires
Isaac Sim to be *fully restarted* after setting, since env vars are read
once at process startup):
```cmd
setx FASTRTPS_DEFAULT_PROFILES_FILE "C:\Users\aravi\.ros\fastdds_unicast_profile.xml"
```
(Isaac Sim's own launch scripts already default `RMW_IMPLEMENTATION` to
`rmw_fastrtps_cpp` and `ROS_DISTRO` to `jazzy` — see
`IsaacSim-6.0.1\setup_ros_env.bat`.)

We also tried a Fast-DDS **Discovery Server** approach first
(`ROS_DISCOVERY_SERVER=192.168.0.36:11811`) as an alternative to the XML
profile — it's a legitimate, more standard approach, but didn't resolve the
issue for us. Left unused/removed in favor of the XML profile.

### (Fallback) Scope the Windows env var to Isaac Sim only

**What / where it's set:** the Windows `setx FASTRTPS_DEFAULT_PROFILES_FILE`
above writes to `HKCU\Environment` — it is **user-level** (only the current
Windows user, not machine-wide or other users) and **Fast-DDS-specific**
(nothing outside eProsima Fast-DDS reads it). It is **not destructive** and is
fully reversible; a missing/malformed profile only makes Fast-DDS log a parse
warning and fall back to defaults, never crash.

**The catch:** it is global *to that user's Fast-DDS apps*, not just to Isaac
Sim. **Every** Fast-DDS/ROS2 program that Windows user launches inherits the
loopback-only, multicast-disabled profile. On this machine that notably
includes **MATLAB (2024b) ROS Toolbox**, which uses its own bundled Fast-DDS
for ROS 2 and picks up the same env var. Symptom to watch for: ROS 2 from
MATLAB on Windows can only see `127.0.0.1` / can't discover nodes on the LAN —
that's this env var, not a bug.

**Fallback if the global `setx` interferes with other Windows DDS/ROS2 work
(e.g. MATLAB):** remove the permanent user-wide variable and instead set it
only for the Isaac Sim launch, via a wrapper `.bat`:

```cmd
:: Remove the permanent user-wide variable (restart Fast-DDS apps afterwards —
:: env vars are read once at process start)
reg delete "HKCU\Environment" /v FASTRTPS_DEFAULT_PROFILES_FILE /f

:: isaac-sim-loopback.bat — confines the loopback profile to Isaac Sim only
@echo off
set "FASTRTPS_DEFAULT_PROFILES_FILE=C:\Users\aravi\.ros\fastdds_unicast_profile.xml"
call "F:\Softwares\IsaacSim-6.0.1\isaac-sim.bat" %*
```

Launch Isaac Sim through `isaac-sim-loopback.bat` instead of `isaac-sim.bat`.
This keeps the loopback profile scoped to Isaac Sim and leaves MATLAB and any
other Windows ROS2 apps on normal (multicast/LAN) discovery. The WSL2 side is
unaffected — its `~/.bashrc` export is already isolated to that shell.

## Isaac Sim side: Action Graph

Built manually via `Create -> Visual Scripting -> Action Graph`, wiring
(node search names in parentheses):

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

- `Isaac Read Joint State Node` -> **Prim Path**: set to the actual
  articulation root.
- `Articulation Controller` -> **Robot Path**: same path, as a plain string.
- `ROS2 Publish Joint State` topic: `joint_states` (default)
- `ROS2 Subscribe Joint State` topic: `joint_command` (default)
- `ROS2 Publish Joint State`'s **Target Prim** left empty — it's an
  alternative to the `Isaac Read Joint State Node` data-wiring approach, not
  used alongside it.

### Gotcha: the articulation root wasn't where expected

Errors in the Console (`Failed to find articulation at '/World/ur5e'`,
`Pattern '/World/ur5e' did not match any articulations`) revealed the actual
articulation root API was applied to a child prim, **`root_joint`** — not
the top-level `/World/ur5e` Xform. This is a known quirk of the URDF
importer when "Fix Base" is enabled. Found the real path by running this in
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

Once the ROS2 connection and OmniGraph wiring were confirmed correct (real
data visibly flowing into `Articulation Controller`'s inputs), the arm
*still* didn't move. Console showed:

```
[omni.physx.tensors.plugin] Simulation view object is invalidated and cannot be used again
[omni.graph.core.plugin] .../articulation_controller: OmniGraph Error: Articulation
  controller failed for prim '/World/ur5e/root_joint': Failed to get DOF position targets
```

This happens because `Articulation Controller` (and joint-state nodes) cache
a handle into the PhysX tensor API, which gets invalidated when the physics
scene is torn down/rebuilt — e.g. from editing the graph or stage *while*
Play is active (which we did, fixing the prim path mid-Play). Fix: **Stop**,
then **Play** again — a clean stop/play cycle. Once done, commands took
effect immediately.

**Going forward: make Action Graph / stage edits while stopped, then press
Play fresh**, to avoid retriggering this.

## ROS2 package: `ur5e_isaac_control`

Located at `src/ur5e_isaac_control/`.

### Node: `joint_command_publisher`

- Subscribes to `/joint_states` to get the arm's current position before
  moving (avoids a jump from a stale default).
- Waits for an actual subscriber match on `/joint_command` before publishing
  (discovery over the unicast-loopback profile isn't instant — a one-shot
  publish before the match completes gets silently dropped).
- Linearly interpolates from current position to a target pose and streams
  it at 60 Hz over a given duration — not a single-shot publish, since a
  single message published before full discovery settles can be lost.

Usage:
```bash
# Move to an explicit target pose (radians, 6 comma-separated values)
ros2 run ur5e_isaac_control joint_command_publisher \
  --positions "0,-1.2,1.2,-1.5,-1.57,0" --duration 3

# Preset two-pose sweep, for a quick sanity check
ros2 run ur5e_isaac_control joint_command_publisher --demo --duration 3

# Preset pick-and-place mimic (approach/pick/lift/transfer/place/lift/home)
ros2 run ur5e_isaac_control joint_command_publisher --pick-place
```

Note: the joints have real position-drive gains (stiffness/damping), so
Isaac Sim's PD controller **holds the last commanded position** on its own —
there's no need to publish continuously to keep the arm in place.

### Node: `joint_controller_gui` (interactive, for learning)

A Tk slider panel — one slider per UR5e joint — for manually driving the arm
and learning its joint ordering, limits, and directions before scripting
motions. Works in WSL2 via WSLg (`DISPLAY=:0`; needs `python3-tk`, already
installed).

- Subscribes to `/joint_states` and **snaps the sliders to the arm's real
  pose on startup** (and via the **Sync to current** button), so enabling
  publish never yanks the arm from a stale value.
- Streams the live slider pose as `sensor_msgs/JointState` to
  `/joint_command` at 30 Hz while **Publish** is checked — drag a slider and
  the arm follows in the viewport.
- Sliders span the UR5e's full +/- 2*pi range; each shows a live radians/degrees
  readout. **Home** button sends the default home pose.
- Status line shows publish on/off and the live `/joint_command` subscriber
  count (`connected` once Isaac Sim's ROS2 Subscribe Joint State node matches).

```bash
# Isaac Sim must be playing first
ros2 run ur5e_isaac_control joint_controller_gui
```

## Everyday restart checklist

1. Open Isaac Sim, load the scene.
2. Press **Play**.
3. If you edited the graph/stage since the last Play, do one **Stop ->
   Play** cycle before sending commands (simulation-view invalidation,
   above).
4. In WSL2: `ros2 topic list` should show `/joint_states` and
   `/joint_command` within a few seconds. If not, double check:
   - Isaac Sim was **fully closed and relaunched** after any env var change
     (env vars are read once at process startup — a running process won't
     pick up a new `setx` value).
   - `FASTRTPS_DEFAULT_PROFILES_FILE` is set on both sides and points at
     matching files.
5. Run `joint_command_publisher` as above.

## Unrelated fixes made along the way

- `~/.bashrc` was sourcing a stale path, `~/matlabros_ws/install/setup.bash`,
  left over from before this workspace was renamed to `isaac_sim_ros` —
  updated to `~/isaac_sim_ros/install/setup.bash`.
- WSL2's default `RMW_IMPLEMENTATION` was `rmw_cyclonedds_cpp`; changed to
  `rmw_fastrtps_cpp` to match Isaac Sim's default and avoid cross-vendor DDS
  interop issues.
