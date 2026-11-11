#!/usr/bin/env bash
# Collect labelled dirt images from several simulated cleaning runs, one random dirt layout
# (and floor colour / light level) per run.
#
#   tools/dirt_dataset/collect_dataset.sh 20            # seeds 1..20, 15 patches each
#   SEED0=100 PATCHES=25 tools/dirt_dataset/collect_dataset.sh 10 world:=test_room.world
#
# Anything after the run count is passed on to `ros2 launch vacuum_bringup vacuum.launch.py`.
# Each run ends when the mission finishes (the recorder shuts the launch down). A run that
# fails does not stop the batch. Output: ~/vacuum_runs/dataset (see package_dataset.py).
set -u
RUNS="${1:-5}"; shift || true
SEED0="${SEED0:-1}"
PATCHES="${PATCHES:-15}"
OUT="${DATASET_DIR:-$HOME/vacuum_runs/dataset}"
TIMEOUT="${RUN_TIMEOUT:-3600}"       # wall-clock seconds before a stuck run is killed

for ((i = 0; i < RUNS; i++)); do
  seed=$((SEED0 + i))
  echo "=== run $((i + 1))/$RUNS  seed $seed ==="
  timeout --signal=INT "$TIMEOUT" ros2 launch vacuum_bringup vacuum.launch.py \
      collect_dataset:=true dataset_dir:="$OUT" autostart:=true \
      dirt:="$PATCHES" dirt_seed:="$seed" dirt_clean:=false dirt_randomize:=true \
      teleop:=none headless:=true rviz:=false "$@" \
    || echo "run for seed $seed ended with an error (continuing)"
  pkill -f "gz sim" 2>/dev/null; sleep 3          # make sure no simulator is left behind
done
echo "done: $(ls -d "$OUT"/ep_* 2>/dev/null | wc -l) episodes in $OUT"
