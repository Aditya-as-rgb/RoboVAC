#!/usr/bin/env bash
#
# Save the live SLAM/Nav2 map into <workspace>/maps — the one canonical place.
#
#   ros2 run vacuum_bringup save_map.sh              # timestamped name
#   ros2 run vacuum_bringup save_map.sh living_room  # maps/living_room.{yaml,pgm}
#
# The map_saver_cli -f prefix is always forced into <workspace>/maps, so a bare
# name cannot scatter maps across the home directory. Anything path-like passed
# as the name is reduced to its basename.
#
# Requires a running map publisher on /map (SLAM toolbox or AMCL with a map).

set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    echo "usage: save_map.sh [NAME]   -> <workspace>/maps/NAME.{yaml,pgm}"
    exit 0
fi

name="${1:-map_$(date +%Y%m%d_%H%M%S)}"
name="$(basename "$name")"
name="${name%.pgm}"
name="${name%.yaml}"
if [[ -z "$name" ]]; then
    echo "error: empty map name" >&2
    exit 1
fi

# Locate the workspace root: the directory holding both src/ and install/.
# Works for a normal install and for --symlink-install (script symlinked into
# src/). Falls back to MAPS_ROOT then ~/vacuum_sim.
self="$(readlink -f "${BASH_SOURCE[0]}")"
root="$(dirname "$self")"
while [[ "$root" != "/" && ! ( -d "$root/src" && -d "$root/install" ) ]]; do
    root="$(dirname "$root")"
done
if [[ "$root" == "/" ]]; then
    root="${MAPS_ROOT:-$HOME/vacuum_sim}"
    echo "warning: workspace not found from $self, using $root" >&2
fi

maps_dir="$root/maps"
mkdir -p "$maps_dir"
out="$maps_dir/$name"

ros2 run nav2_map_server map_saver_cli -f "$out"
echo "saved: $out.yaml and $out.pgm"
