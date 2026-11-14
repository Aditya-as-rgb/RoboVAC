#!/usr/bin/env bash
#
# bootstrap.sh - fresh clone to a working simulation, in one command.
#
#   ./bootstrap.sh                 # ROS dependencies, Python extras, colcon build
#   ./bootstrap.sh --desktop       # ...and install the control panel in the app menu
#   ./bootstrap.sh --no-build      # dependencies only (build later by hand)
#   ./bootstrap.sh --no-pip        # skip the pip extras (numpy/onnxruntime/PySide6)
#
# Idempotent: it checks before installing and re-running is cheap. Needs sudo for the
# apt/rosdep step and network access for rosdep. ROS 2 itself is not installed here -
# if /opt/ros/<distro>/setup.bash is missing the script says where to get it and stops.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DO_BUILD=1
DO_PIP=1
DO_DESKTOP=0

usage() {
    sed -n '2,10p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

for arg in "$@"; do
    case "$arg" in
        --desktop)   DO_DESKTOP=1 ;;
        --no-build)  DO_BUILD=0 ;;
        --no-pip)    DO_PIP=0 ;;
        -h|--help)   usage; exit 0 ;;
        *)           echo "unknown option: $arg (try --help)" >&2; exit 2 ;;
    esac
done

if [[ ! -d "$HERE/src" ]]; then
    echo "error: $HERE has no src/ - run this from the workspace root" >&2
    exit 1
fi

# `source` under `set -u` trips on the setup scripts' unset variables.
src_setup() { set +u; # shellcheck disable=SC1090
    source "$1"; set -u; }

step() { printf '\n==> %s\n' "$*"; }

if [[ "$(id -u)" == 0 ]]; then
    echo "error: run as your normal user, not root (colcon and pip write to your home)" >&2
    exit 1
fi

# --- 1. ROS 2 -------------------------------------------------------------------------
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    cat >&2 <<EOF
error: ROS 2 $ROS_DISTRO is not installed (/opt/ros/$ROS_DISTRO/setup.bash missing).

Install it first (Ubuntu 24.04, desktop or ros-base):
  https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

Then run this script again. Set ROS_DISTRO=humble ./bootstrap.sh for another distro -
Jazzy is the tested one.
EOF
    exit 1
fi
step "ROS 2 $ROS_DISTRO found"
src_setup "/opt/ros/$ROS_DISTRO/setup.bash"

# A venv/conda python3 earlier on PATH breaks colcon: ament's cmake scripts run `python3`
# and need catkin_pkg from the system site-packages. Use the system one for the build.
if ! python3 -c 'import catkin_pkg' >/dev/null 2>&1; then
    if /usr/bin/python3 -c 'import catkin_pkg' >/dev/null 2>&1; then
        step "python3 on PATH is not the ROS one - putting /usr/bin first for this script"
        export PATH="/usr/bin:/bin:$PATH"
    else
        echo "error: no python3 here can import catkin_pkg (sudo apt install python3-catkin-pkg)" >&2
        exit 1
    fi
fi

# --- 2. declared dependencies (ROS packages + apt system packages) ---------------------
if ! command -v rosdep >/dev/null 2>&1; then
    echo "error: rosdep is missing: sudo apt install python3-rosdep" >&2
    exit 1
fi
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    step "Initialising rosdep (sudo)"
    sudo rosdep init || true                 # already initialised is fine
fi
step "Updating the rosdep index (network)"
rosdep update || echo "warning: rosdep update failed - using the existing index"
step "Installing everything package.xml declares (sudo apt through rosdep)"
rosdep install --from-paths src --ignore-src -r -y

# --- 3. Python extras in the system interpreter (the one rclpy lives in) ---------------
if (( DO_PIP )); then
    PY=/usr/bin/python3
    if ! "$PY" -m pip --version >/dev/null 2>&1; then
        echo "warning: $PY has no pip (sudo apt install python3-pip); skipping the extras" >&2
    else
        extras=()
        "$PY" - <<'EOF' || extras+=('numpy<2')
import sys, numpy
sys.exit(0 if numpy.__version__.split('.')[0] == '1' else 1)
EOF
        "$PY" -c 'import onnxruntime' >/dev/null 2>&1 || extras+=(onnxruntime)
        "$PY" -c 'import PySide6' >/dev/null 2>&1 || extras+=(PySide6)
        if (( ${#extras[@]} )); then
            step "pip (user site): ${extras[*]}"
            "$PY" -m pip install --user --break-system-packages "${extras[@]}"
        else
            step "Python extras already present (numpy<2, onnxruntime, PySide6)"
        fi
    fi
fi

# --- 4. build -------------------------------------------------------------------------
if (( DO_BUILD )); then
    step "colcon build --symlink-install"
    colcon build --symlink-install
    src_setup "$HERE/install/setup.bash"
fi

# --- 5. optional desktop entry --------------------------------------------------------
if (( DO_DESKTOP )); then
    step "Installing the desktop launcher (no sudo, ~/.local only)"
    if [[ ! -f "$HERE/install/setup.bash" ]]; then
        src_setup "$HERE/install/setup.bash"
    fi
    ros2 run vacuum_gui vacuum_gui_install_desktop
fi

# --- 6. what to do now ----------------------------------------------------------------
shell_name="$(basename "${SHELL:-bash}")"
setup_file="setup.bash"
[[ "$shell_name" == "zsh" ]] && setup_file="setup.zsh"

step "Ready"
cat <<EOF
Source the workspace in each new shell:
  source /opt/ros/$ROS_DISTRO/$setup_file && source install/$setup_file

Then, from this directory:
  ros2 launch vacuum_bringup vacuum.launch.py                # sim + Nav2 + RViz + coverage
  ros2 launch vacuum_bringup vacuum.launch.py autostart:=true # ...and start cleaning
  ros2 launch vacuum_gui gui.launch.py                       # the control panel
  /usr/bin/python3 -m pytest src/*/test -q       # 242 tests, no ROS or display needed

Maps are saved to <workspace>/maps by \`ros2 run vacuum_bringup save_map.sh\`, and the
launch files look there first, so a saved map is picked up with no extra arguments.
EOF