# Archive: Isaac Sim (Windows) ↔ ROS2 (WSL2) Connection Issues

> **This file is historical documentation only.** It records problems hit and fixes tried while
> running Isaac Sim on Windows against ROS2 Jazzy in WSL2 — a setup this project has since
> abandoned in favor of dual-boot Ubuntu (see [PLAN.md](PLAN.md)). **Nothing here should be
> consulted for troubleshooting the current setup** — it exists purely as a record of what was
> tried and why it didn't work out. If dual-boot Ubuntu is ever abandoned and Windows+WSL2 is
> revisited, treat this as a starting point, not a solved problem.

## Environment at the time

- Isaac Sim 6.0.1 ran natively on **Windows** (`F:\Softwares\IsaacSim-6.0.1`), with its own bundled
  ROS2 Jazzy libraries (`isaacsim.ros2.bridge` / `isaacsim.ros2.core`).
- ROS2 Jazzy ran natively inside **WSL2** (Ubuntu 24.04).
- WSL2 networking mode was **mirrored** (`.wslconfig`: `networkingMode=mirrored`), so WSL2 shared
  Windows' IP addresses and loopback interface rather than getting its own NATed IP.

## The core problem: WSL2 mirrored mode breaks DDS discovery

ROS2's default discovery (Fast-DDS SPDP) relies on **UDP multicast**. This is a known,
NVIDIA-confirmed bug: UDP multicast forwarding across the Windows↔WSL2 boundary is unreliable
under mirrored networking mode, even though ordinary unicast traffic (ping, TCP, plain UDP) passes
through fine. Result: `ros2 topic list` in WSL2 only ever showed the daemon's own default topics
(`/parameter_events`, `/rosout`) — nothing from Isaac Sim, regardless of how correct the OmniGraph,
prim paths, or firewall rules were.

NVIDIA's forums stated the *officially supported* path for Windows + ROS2 is **WSL2 NAT mode +
`netsh interface portproxy`** port forwarding, not mirrored mode. We deliberately avoided that
(more disruptive, changes IP behavior for all of WSL2) and used a documented community workaround
instead.

## The fix that was tried: unicast-only Fast-DDS profile

A custom Fast-DDS XML profile disabling multicast entirely, forcing **all** DDS traffic —
discovery *and* message data — over loopback (`127.0.0.1`), which mirrored-mode WSL2 shares
reliably with Windows.

File: `~/.ros/fastdds_unicast_profile.xml` (WSL2), with an identical copy at
`C:\Users\aravi\.ros\fastdds_unicast_profile.xml` for Isaac Sim to read.

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

1. **`metatrafficUnicastLocatorList` alone isn't enough.** It only routes *discovery* traffic. It's
   possible for discovery to succeed (topics show up in `ros2 topic list`, subscriber counts look
   right) while actual message *data* still silently fails to arrive, because Fast-DDS auto-picks a
   separate locator for user data unless `defaultUnicastLocatorList` is also set. This exact
   asymmetry is what caused `/joint_states` (Windows→WSL2) to work before `/joint_command`
   (WSL2→Windows) did.
2. A community-posted version of this XML (NVIDIA forums) included
   `<use_SIMPLE_EndpointDiscoveryProtocol>` and `<use_WriterLivelinessProtocol>` tags inside
   `<discovery_config>` — invalid elements in this Fast-DDS XSD version, causing a silent XML parse
   failure (visible via `ros2 topic list --no-daemon` stderr) and a silent fallback to defaults.
   Removed them; `<discoveryProtocol>` alone was sufficient.

### Environment variables required (both sides)

**WSL2** (`~/.bashrc`):
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # must match Isaac Sim's default
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_unicast_profile.xml
```

**Windows** (`setx`, requires Isaac Sim to be *fully restarted* after setting — env vars are read
once at process startup):
```cmd
setx FASTRTPS_DEFAULT_PROFILES_FILE "C:\Users\aravi\.ros\fastdds_unicast_profile.xml"
```

A Fast-DDS **Discovery Server** approach (`ROS_DISCOVERY_SERVER=192.168.0.36:11811`) was also tried
as an alternative to the XML profile — legitimate and more standard, but didn't resolve the issue.
Abandoned in favor of the XML profile.

### Fallback: scoping the Windows env var to Isaac Sim only

The global `setx` above is user-level and Fast-DDS-specific, but applies to **every** Fast-DDS/ROS2
program that Windows user launches — notably **MATLAB (2024b) ROS Toolbox**, which uses its own
bundled Fast-DDS and picked up the same variable, losing LAN node discovery. Fallback tried: remove
the global variable and scope it to Isaac Sim only via a wrapper `.bat`:

```cmd
reg delete "HKCU\Environment" /v FASTRTPS_DEFAULT_PROFILES_FILE /f

:: isaac-sim-loopback.bat
@echo off
set "FASTRTPS_DEFAULT_PROFILES_FILE=C:\Users\aravi\.ros\fastdds_unicast_profile.xml"
call "F:\Softwares\IsaacSim-6.0.1\isaac-sim.bat" %*
```

## Outcome of the unicast profile: worked for topics, not for MoveIt

With the profile in place, plain topic traffic worked end-to-end — live `/joint_states` streaming
and `/joint_command` driving the arm, including a full pick-and-place mimic sequence. This is what
the deleted `SETUP_NOTES.md` documented as "done."

**MoveIt broke it, via an A/B test:**

| Configuration | Result |
|---|---|
| Profile active | `move_group` plans successfully, but the result **never reaches RViz** — panel hangs on "Planning…"; `get_planning_scene` also times out |
| Profile removed (WSL2-local only) | Plan completes in **0.017 s**, preview animates, all fine |

**Topics worked; request/reply (services, and the action result fetch) did not.** Suspected cause:
the profile pins every participant to one `defaultUnicastLocatorList` locator with no port, so with
5+ nodes the participants collide instead of each getting a distinct port — consistent with a
2-process setup always working and only the multi-node MoveIt stack failing.

### Other things tried and ruled out

1. `useBuiltinTransports` → `true` — no effect.
2. Stripping the profile from RViz only — TF still crossed, RPC still failed, so the problem
   followed `move_group`, not the requester.
3. **Fast-DDS Discovery Server** on `127.0.0.1:11811` — dead end. A raw UDP listener proved Isaac
   Sim *did* honour `ROS_DISCOVERY_SERVER` and its packets *did* cross, and the server worked for
   WSL2-local nodes — but Isaac Sim's topics never appeared. Likely a Fast-DDS version mismatch
   between Isaac Sim's bundled DDS and Jazzy's.
4. **Isaac Sim inside WSL2** (sidestepping the boundary entirely) — impossible on this machine. No
   NVIDIA Vulkan support in WSL2: no `nvidia_icd.json` and no Vulkan library to point an ICD at.
   Known NVIDIA regression from driver ~581.xx; this machine was on 610.88. Isaac Sim's RTX
   renderer requires hardware Vulkan.

## The decisive discovery

NVIDIA marks **WSL2 as "(Deprecated)"** for Isaac Sim + ROS2 and recommends **native Windows ROS2
Jazzy via Pixi**, which uses **Zenoh** (`rmw_zenoh_cpp`) instead of Fast-DDS — removing both the
host boundary and the middleware that failed.

Docs referenced at the time:
<https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_ros_other_platforms.html>

## Two migration paths considered

**Plan A — Pixi, native Windows (Zenoh).** Investigated as the lower-risk option: install Pixi,
clone NVIDIA's `IsaacSim-ros_workspaces` repo, `pixi install` to get ROS2 Jazzy + MoveIt +
compilers, add `ros-jazzy-ur-moveit-config` / `ros-jazzy-ur-description`, build, and run Isaac Sim
via `pixi run sim`. Verified package availability (RoboStack `jazzy` channel) for
`ros-jazzy-moveit`, `ros-jazzy-ur-moveit-config` (exact version in use, 3.8.0),
`ros-jazzy-ur-description`, `ros-jazzy-moveit-servo` — all present for `win-64`. **Not pursued to
completion** — superseded by the dual-boot decision below before being fully tried.

**Plan B — dual-boot Ubuntu 24.04.** The path actually taken. Kept as fallback-turned-primary once
it became clear a clean native-Linux setup removed the WSL2/Windows boundary problem entirely
rather than replacing one middleware workaround with another, less-proven one.

### Plan B execution notes (for the historical record)

Hardware: Lenovo Legion 5 16IRX9, Windows 11 Home build 26200, RTX 4060 Laptop (8 GB VRAM), all
four disk volumes **BitLocker-encrypted**.

Key steps taken, in order:
1. Saved all four BitLocker recovery keys off-machine before touching anything.
2. Disabled Windows Fast Startup (was on; would have made Linux see a dirty NTFS).
3. Fully decrypted the target volume (`manage-bde -off F:`) — BitLocker-encrypted volumes can't be
   shrunk, and Linux can't read encrypted NTFS volumes at all.
4. Backed up the WSL2 workspace and copied the USD scene off `F:` before repartitioning.
5. Suspended (not removed) BitLocker protection on `C:` for the duration of the install, to avoid a
   TPM-triggered recovery-key prompt from the firmware/boot-order change.
6. Shrank the decrypted volume and installed Ubuntu into the freed space, with **its own separate
   EFI System Partition** on the second disk — so Windows' and Ubuntu's bootloaders never share a
   partition and Windows updates can't clobber GRUB.
7. Post-install: installed the **proprietary** NVIDIA driver (not the `-open` variant — reports of
   instability on this GPU class), verified Vulkan support (`vulkaninfo --summary` must show
   NVIDIA, not `llvmpipe`), used Xorg rather than Wayland, and completed the blue MOK-enrollment
   screen for the driver under Secure Boot.
8. Installed ROS2 Jazzy, MoveIt2, and the UR config packages via `apt` — no Pixi/Zenoh needed,
   since discovery is native.
9. Deleted the WSL2-era DDS workarounds: the `FASTRTPS_DEFAULT_PROFILES_FILE` export, the profile
   XML file itself, the Windows `setx` value, and any `netsh portproxy` rules.

## Where this leaves the project

The project now runs entirely on native Ubuntu — see [PLAN.md](PLAN.md) for current status and
[EXPLAINED.md](EXPLAINED.md) for how the pieces work. Everything in this file describes a
discarded approach and a migration that has already happened; it is kept only so the reasoning
behind the current setup isn't lost.
