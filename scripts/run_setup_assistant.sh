#!/usr/bin/env bash
# Launch the MoveIt Setup Assistant with the VSCode-snap environment stripped.
#
# Running it directly from a VSCode-spawned terminal dies immediately with:
#
#   /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0:
#   undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE
#
# Cause: this shell inherits VSCode's snap environment, so GTK_PATH, LOCPATH,
# XDG_DATA_DIRS etc. point inside /snap/code/<rev>/usr. Qt apps loading GTK/
# locale plugins then pull in the snap's bundled libpthread ahead of the system
# one. It is the same failure that launch/moveit.launch.py works around for
# rviz2 -- but that workaround is scoped to the rviz2 process, so any other GUI
# binary (like this one) hits it afresh.
#
# Stripping the vars only for this process leaves everything else untouched.
#
# Usage:  ./scripts/run_setup_assistant.sh
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec env \
  -u GTK_PATH -u GTK_EXE_PREFIX -u GDK_PIXBUF_MODULEDIR -u GDK_PIXBUF_MODULE_FILE \
  -u GIO_MODULE_DIR -u GIO_LAUNCHED_DESKTOP_FILE -u GIO_LAUNCHED_DESKTOP_FILE_PID \
  -u GSETTINGS_SCHEMA_DIR -u GTK_IM_MODULE_FILE -u LOCPATH \
  -u XDG_DATA_DIRS -u XDG_DATA_HOME -u XDG_CONFIG_DIRS \
  bash -c "
    source /opt/ros/jazzy/setup.bash
    source '$WS/install/setup.bash' 2>/dev/null || true
    echo 'Setup Assistant starting. Load this file:'
    echo '    $WS/src/ur5e_isaac_control/urdf/ur5e_gripper.urdf.xacro'
    echo
    ros2 launch moveit_setup_assistant setup_assistant.launch.py
  "
