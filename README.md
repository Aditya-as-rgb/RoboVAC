# vacuum_sim

ROS 2 **Jazzy** + Gazebo (gz-sim) simulation of a round robot vacuum: SLAM, Nav2,
teleop, bumpers, coverage cleaning with a run report, simulated dirt, and a vision model
that finds dirt on the floor. Runs natively on Ubuntu 24.04, no Docker, no `kaiaai` packages.

The goal of the project is premium-vacuum features (SLAM, Nav2, coverage planning) on
budget-tier hardware. The "innovation" part is a **verify-and-reclean loop**: after a clean the
robot looks at the floor with its camera, finds dirt that is still there, and goes back to it.

The robot model, worlds and Nav2 tuning are derived from the open-source
[OOMWOO](https://github.com/makerspet/oomwoo) project by Maker's Pet / KAIA.AI
(Apache-2.0). See [NOTICE](NOTICE). The plan is to start with the OOMWOO One URDF
unchanged and later replace it with our own design.

## Layout

| Package / folder | Contents |
|---|---|
| `vacuum_description` | `urdf/` xacro (body, wheels, LiDAR, ToF, side ranges, IMU, bumpers, front floor camera, gz plugins), `config/gz_bridge.yaml` (gz <-> ROS topic bridge), `config/ekf.yaml` (for a real robot, unused in sim), RViz configs |
| `vacuum_gazebo` | `worlds/living_room.world`, `models/`, `maps/`, `launch/sim.launch.py`, `scripts/` (dirt generator `dirt_world.py`, brush `dirt_cleaner.py`, auto-labelling `dirt_labels.py`, dataset recorder `dirt_dataset.py`) |
| `vacuum_bringup` | `launch/vacuum.launch.py` (everything in one command), `launch/navigation.launch.py` (SLAM or AMCL + Nav2 + RViz), `config/navigation.yaml`, `config/explore_lite.yaml` (for later) |
| `vacuum_coverage` | wall-pass + lane-sweep coverage mission on top of Nav2 (`coverage_mission`), cleaned-area overlay and percentage (`coverage_meter`), re-clean pass, run report, offline planner/preview (`coverage_preview`), unit tests |
| `vacuum_teleop` | `launch/teleop_joy.launch.py` + `config/teleop_joy.yaml` — gamepad teleop (`joy` + `teleop_twist_joy` -> `/cmd_vel`) |
| `vacuum_gui` | PySide6 control panel: launch manager, map/camera/health views drawn from ROS topics, mission and teleop controls, dataset and experiment tooling (see [Control panel](#control-panel-vacuum_gui)) |
| `tools/dirt_dataset` | `collect_dataset.sh` (batch of labelled runs), `package_dataset.py` (train/val/test split + zip), `train_dirt_yolo.ipynb` (Colab training) |
| `models/dirt_detector` | trained dirt detector (`.pt` and `.onnx`) and its README |

## Install dependencies

```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher python3-colcon-common-extensions \
  python3-rosdep python3-numpy python3-yaml python3-matplotlib python3-pil python3-pytest
```

Training the detector needs Ultralytics, but only in Colab (or any separate Python
environment); the ROS side does not.

## Build

```bash
cd <your workspace>            # the folder that contains src/
source /opt/ros/jazzy/setup.bash      # zsh users: setup.zsh, here and below
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run everything with one command

```bash
source install/setup.bash        # zsh: install/setup.zsh
ros2 launch vacuum_bringup vacuum.launch.py                 # sim + Nav2 (AMCL on the saved map) + RViz + gamepad + coverage
ros2 launch vacuum_bringup vacuum.launch.py autostart:=true # ...and start cleaning by itself
```

Arguments: `slam:=True` (map while driving, instead of AMCL), `map:=/path/map.yaml`,
`rviz:=false`, `headless:=true`, `teleop:=none`, `coverage:=false`, `meter:=false`,
`autostart:=true`, `world`, `x_pose`/`y_pose`, `odom_source`, `nav_delay` (seconds
Gazebo gets before Nav2 starts, default 8), plus the dirt and dataset arguments below.
Without `autostart`, start or stop a clean from another terminal:

```bash
ros2 service call /coverage_mission/plan  std_srvs/srv/Trigger   # draw the plan in RViz only
ros2 service call /coverage_mission/start std_srvs/srv/Trigger
ros2 service call /coverage_mission/stop  std_srvs/srv/Trigger
```

## Run step by step (one terminal each, source `install/setup.bash` in every one)

```bash
# 1. Gazebo + robot (living room)
ros2 launch vacuum_gazebo sim.launch.py

# 2. SLAM + Nav2 + RViz: map while you drive / send goals
ros2 launch vacuum_bringup navigation.launch.py

# 3. Drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# ...or with a gamepad (see "Gamepad teleop" below)
ros2 launch vacuum_teleop teleop_joy.launch.py

# 4. Save the map when done (always into <workspace>/maps)
ros2 run vacuum_bringup save_map.sh living_room
```

Navigate on a saved map instead of mapping (step 2 then becomes this):

```bash
ros2 launch vacuum_bringup navigation.launch.py slam:=False \
  map:=<workspace>/maps/living_room.yaml     # e.g. $HOME/RoboVAC/maps/living_room.yaml
```

`save_map.sh` derives the workspace root from its own install path, so it always
writes to `<workspace>/maps`; pass only a name, never a path. Without an argument
it uses a timestamp. This replaces the raw `map_saver_cli -f ~/maps/...` call.

AMCL is seeded at the spawn pose (-2.0, -0.5). If you change `x_pose`/`y_pose` in
`sim.launch.py`, change `initial_pose` in `vacuum_bringup/config/navigation.yaml`
too, or use "2D Pose Estimate" in RViz.

Useful `sim.launch.py` arguments: `headless:=true` (no GUI), `software_gl:=true`
(force software rendering if the Gazebo window or headless mode crashes on OpenGL;
headless already switches to software rendering by itself when there is no `/dev/dri`),
`enable_tof:=false enable_cameras:=false` (faster), `odom_source:=robot_wheels` (wheel
odometry that drifts, instead of ground truth), `x_pose`, `y_pose`, `world`.

## Coverage (wall pass + lane sweep)

`vacuum_coverage` plans on the occupancy grid of the saved map and drives the plan with
Nav2, so use it with AMCL (`slam:=False`), not while the map is still changing.

![coverage plan on the living-room map](docs/coverage_plan.png)

*Offline preview on the saved living-room map: orange wall loops, blue sweep lanes, shaded
reachable floor. 96.7 % of the reachable area, 75.7 % of all free floor, 0.197 m clearance.*

1. **Wall pass.** Grow the obstacles by the robot radius plus a margin; the boundary of the
   remaining free space is a set of closed loops: one round the walls and one round each piece
   of furniture. Each loop is smoothed and sent to Nav2 as a `FollowPath` goal.
2. **Lane sweep.** Parallel lanes `lane_spacing` apart through what is left (along the
   longer side of the room), cut wherever they hit furniture, ordered nearest-first. Hops
   between pieces use `NavigateToPose`; an aborted piece is retried from the nearest point
   on it, then skipped after `max_retries`.
3. **Self-check.** Before it moves, the planner drives its own plan through an offline model
   of the Nav2 controller (`CoveragePath` in `navigation.yaml`: 0.25 m lookahead, 0.25 m/s)
   and widens `wall_margin` until the simulated path keeps at least `robot_radius + 2 cm`
   from every obstacle. On the living-room map it ends at a 0.11 m margin and a simulated
   clearance of 0.197 m (robot radius 0.1745 m).
4. **Re-clean pass.** When the plan is done, the mission compares what the coverage meter
   marked as cleaned with the reachable floor, picks the fewest spots that cover the missed
   cells (greedy set cover), drives to each one and cleans it, and repeats up to
   `reclean_passes` times (default 2). This uses only the map and the robot's own track, no
   camera. It is the baseline the vision-guided version is compared with.

The meter marks every floor cell within `clean_radius` (0.22 m) of where the robot has
been. In RViz: *Cleaned area* (blue, `/coverage_map`), *Coverage plan* (orange wall loops,
blue lanes) and the text *Cleaned: xx %*. Numbers also on `/coverage/percent` and
`/coverage/percent_free`; `ros2 service call /coverage_meter/reset std_srvs/srv/Trigger` clears them.

**What the percentage means.** `/coverage/percent` is the share of the floor a robot of this
size can reach and clean. The speckled coffee-table/rug area is excluded because the robot
cannot fit between the table legs; `/coverage/percent_free` counts that area too (the
planner reports 96.7 % vs 75.7 % in simulation). Quote both numbers and say which is which.

Plan preview without ROS (needs numpy, matplotlib, pyyaml) — this is how `docs/coverage_plan.png`
was made:

```bash
cd src/vacuum_coverage
python3 -m vacuum_coverage.preview --map ../vacuum_gazebo/maps/living_room.yaml --out plan.png --simulate
python3 -m pytest test        # planner, controller model, re-clean, report and mission tests
```

Tuning: `src/vacuum_coverage/config/coverage.yaml` (lane spacing, `reclean`, `reclean_passes`,
`reclean_slack`, `reclean_min_area`, report settings). Keep `pursuit_lookahead`/`pursuit_speed`
equal to `CoveragePath` in `navigation.yaml`, or the offline safety check no longer
describes the real controller. The offline model is an approximation of Nav2: check the
first real run in RViz and raise `safety` or `wall_margin` if the robot grazes furniture.
The sofa and tables are seen by the LiDAR; check that they also have collision geometry in
the world if the robot drives through them.

Running the mission on its own, with the sim and Nav2 already up (the step-by-step path):

```bash
ros2 launch vacuum_coverage coverage.launch.py                 # wait for /coverage_mission/start
ros2 launch vacuum_coverage coverage.launch.py autostart:=true  # clean straight away
```

### Run report

Every mission writes a report folder under `~/vacuum_runs/` (`report_dir` in `coverage.yaml`):

| File | What it holds |
|---|---|
| `summary.json` | totals: time, distance, coverage before/after each pass, pieces done/skipped, re-clean spots |
| `coverage.csv` | coverage percentage over time (sampled every `report_period` seconds) |
| `pieces.csv` | one row per wall loop / lane / re-clean spot: pass, start and end time, status |
| `report.png` | one picture of the coverage curve, the path driven and the spots revisited |

Use `summary.json` for the numbers in a report and `report.png` for slides. Compare runs by
keeping each run's folder.

## Dirt in the simulation

`dirt:=N` scatters N visual-only dirt patches (no collision, the robot drives over them) on
floor the robot can reach, placed from the map. The same `dirt_seed` always gives the same
layout. Four kinds, each with its own look: **crumbs**, **dust** (translucent), **spill**
(glossy, always needs several passes) and **hair** (thin strands).

```bash
ros2 launch vacuum_bringup vacuum.launch.py dirt:=15 dirt_seed:=3 autostart:=true
```

| Argument | Meaning |
|---|---|
| `dirt:=N` | number of patches (0 = none, the default) |
| `dirt_seed:=S` | same seed = same layout |
| `dirt_scale:=2.0` | patch size multiplier (bigger = easier for a camera) |
| `dirt_stubborn:=0.3` | share of patches that need several passes (spills always do) |
| `dirt_clean:=false` | leave dirt on the floor (default `true`: the brush removes what it drives over) |
| `dirt_brush:=0.20` | brush radius in metres |
| `dirt_randomize:=true` | random floor colour and light level for this seed (for training data) |
| `dirt_map:=...` | map used to find reachable floor (defaults to the world's own) |

Ground truth is written to `~/vacuum_runs/dirt/<world>_dirt<N>_seed<S>.json` (and
`current_layout.json`). The brush model removes a patch when enough brush travel passes over
it; stubborn patches need more. Removed patches disappear in Gazebo through the
`/world/default/remove` service; if they do not disappear, see the note at the top of
`vacuum_gazebo/scripts/dirt_cleaner.py`. What is left is on `/dirt/remaining`, `/dirt/state`
(JSON list of remaining patch names) and in `~/vacuum_runs/dirt/dirt_summary_*.json`. In RViz,
`/dirt/markers` shows the ground-truth dirt that is left (the answer key, not what a camera sees).

Preview a layout without starting Gazebo:

```bash
python3 src/vacuum_gazebo/scripts/dirt_world.py --help
```

## Vision: finding dirt with the floor camera

The robot has one RGB floor camera at the front (640x480, 120 degree horizontal FoV, about
7 cm above the floor, tilted down about 38 degrees, with a little sensor noise). It looks at the
floor ahead, so dirt appears wide and short, and patches more than about 1 m away are only a few
pixels tall. Switch it on with `cameras:=true` (it is on automatically while collecting data).

### Training data from the simulator (no hand labelling)

Gazebo knows where every dirt patch is and ROS knows where the camera is, so every camera frame
can be labelled automatically: each patch outline is projected into the image and its bounding
box written in YOLO format. Frames where a label would be unreliable (patch cut by the image edge,
partly hidden, or too small to judge) are dropped rather than labelled wrongly.

```bash
# 1. record labelled frames from N cleaning runs, each with a new dirt layout, floor colour and light
tools/dirt_dataset/collect_dataset.sh 20          # SEED0=100 PATCHES=25 ... to change seeds / patch count

# 2. split by episode (not by frame) and zip for Colab
python3 tools/dirt_dataset/package_dataset.py ~/vacuum_runs/dataset ~/vacuum_runs/dirt_yolo --classes kind --zip
#    --classes single  = one "dirt" class instead of crumbs / dust / spill / hair
```

A single run on its own:

```bash
ros2 launch vacuum_bringup vacuum.launch.py collect_dataset:=true autostart:=true \
  dirt:=15 dirt_seed:=1 dirt_clean:=false dirt_randomize:=true teleop:=none headless:=true
```

Keep `dirt_clean:=false` while collecting (the dirt stays on the floor so the camera sees it
from every side) and keep the default ground-truth odometry: the labels are in the `map` frame, and
with AMCL the pose drifts and the boxes would drift with it. Output goes to `~/vacuum_runs/dataset/`
(`classes.txt`, and per episode `images/`, `labels/`, `meta.csv`, `episode.json`).

### Training and the current model

`tools/dirt_dataset/train_dirt_yolo.ipynb` trains YOLO11n in Google Colab (GPU runtime), scores it
on the held-out test episodes, tries real photos if you add a `real_photos/` folder, and exports
ONNX and ncnn. The trained model is in `models/dirt_detector/`.

Current model: 10 simulated episodes, 3040 train / 1062 val / 1296 test images, split by episode.
Results on the held-out test episodes:

| Class | Precision | Recall | mAP50 |
|---|---|---|---|
| all | 0.72 | 0.63 | 0.68 |
| crumbs | 0.77 | 0.77 | 0.74 |
| dust | 0.93 | 0.29 | 0.54 |
| spill | 0.63 | 0.83 | 0.84 |
| hair | 0.58 | 0.60 | 0.60 |

Dust (translucent) is often missed; crumbs and spills are found reliably. The spill figure comes
from only 66 test boxes. **These are simulation numbers only.** The model has not yet been tested on
real floor photos, which is the number that decides how far it can be trusted.

## Tests

```bash
python3 -m pytest src/vacuum_coverage/test src/vacuum_gazebo/test -q
```

No ROS or Gazebo is needed for these: planner, controller model, re-clean, report, mission
sequencing, dirt generator, brush model, camera labelling, dataset recorder and packaging.

## Gamepad teleop

Instead of `teleop_twist_keyboard`, the robot can be driven with an
Xbox-compatible gamepad (tested with an EvoFox One S over Bluetooth, which the
kernel's `xpad` driver exposes as `/dev/input/js0`). The wrapper package is
`vacuum_teleop`.

```bash
ros2 launch vacuum_teleop teleop_joy.launch.py
```

Hold **LB** to enable (deadman), drive with the **left stick**, **RB** for turbo.
Speeds live in `vacuum_teleop/config/teleop_joy.yaml` (0.30 m/s and 1.20 rad/s
by default, 0.50 / 2.50 with turbo) — keep them at or below the Nav2 limits in
`navigation.yaml`.

If the pad isn't picked up, list what SDL sees and pass the index:

```bash
ros2 run joy joy_enumerate_devices
ros2 launch vacuum_teleop teleop_joy.launch.py joy_dev:=0
```

Both teleop nodes publish `/cmd_vel`, so use one at a time, and don't drive with
the pad while Nav2 is executing a goal (remap `joy_vel` and arbitrate with
`twist_mux` if you need both).

## Interface contract (what your own URDF must provide)

Nav2, SLAM and the bridge rely on these. Keep them and everything else keeps working.

| Item | Value |
|---|---|
| TF chain | `map -> odom -> base_footprint -> base_link -> base_scan` (+ wheel, sensor links, `camera_link -> camera_optical_frame`) |
| Topics out | `/scan` (frame `base_scan`), `/odom`, `/imu`, `/tf`, `/joint_states`, `/clock`, `/bumper_left/contact`, `/bumper_right/contact`, `/camera/image` + `/camera/camera_info` (when the camera is enabled) |
| Topic in | `/cmd_vel` (`geometry_msgs/Twist`), a diff-drive plugin |
| gz model name | `vacuum` (set by `sim.launch.py`, referenced in `config/gz_bridge.yaml`) |
| gz world name | `default` (also referenced in `gz_bridge.yaml` and by the dirt removal service) |
| Footprint | Nav2 radius/footprint in `navigation.yaml` is tuned for ~0.349 m diameter; update it when your body size changes |
| Camera | optical frame follows REP-103 (z forward, x right, y down); the labelling code reads the camera pose from TF and the intrinsics from `camera_info` |

The bumpers are easy to break silently. Read the three conditions in the
`gz_bridge.yaml` comments and `plugins.xacro`: contact sensors reference the
post-URDF-to-SDF collision names, the bridge uses the scoped gz topic, and the
world must load `gz-sim-contact-system`.

### Swapping in your own robot

1. Copy `vacuum_description` to a new package (e.g. `myvac_description`) and edit
   `urdf/params.xacro` (dimensions, wheels, sensors) and `urdf/robot.urdf.xacro`.
2. Launch with `ros2 launch vacuum_gazebo sim.launch.py description_pkg:=myvac_description`.
3. If the robot's collision layout changes, regenerate the bumper collision names:
   `xacro urdf/robot.urdf.xacro | gz sdf -p /dev/stdin | grep 'collision name'`.
4. Update the Nav2 footprint and inflation in `vacuum_bringup/config/navigation.yaml`.

## Troubleshooting

- **Gazebo crashes headless (exit 139, EGL / `driCreateNewScreen3`).** Check that
  `MESA_LOADER_DRIVER_OVERRIDE` is not set to `llvmpipe` in your shell. Headless now uses software
  rendering only when `/dev/dri` is missing; `software_gl:=true` forces it.
- **ROS nodes die on import (numpy / rclpy not found).** A virtualenv Python is shadowing
  `/usr/bin/python3` on `PATH`. Launch with `PATH=/usr/bin:/bin:$PATH ros2 launch ...`, or deactivate
  the venv first.
- **`source install/setup.bash` fails under `set -u`.** Source it with `set +u` around it.

## Control panel (`vacuum_gui`)

One PySide6 window that starts and stops the whole stack, draws everything the robot sees
and knows from ROS topics, and drives it. Gazebo and RViz are **not** embedded: the map,
camera, LiDAR, plans and dirt are drawn natively, so the simulation can run headless (about
half the GPU load of a Gazebo window) and RViz becomes optional. The buttons for the real
Gazebo and RViz windows are still there.

```bash
# once
/usr/bin/python3 -m pip install --user --break-system-packages PySide6

cd ~/vacuum_sim && source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select vacuum_gui
source install/setup.zsh

ros2 run vacuum_gui vacuum_gui                  # the panel
ros2 launch vacuum_gui gui.launch.py            # same thing as a launch file
ros2 run vacuum_gui vacuum_gui_install_desktop  # optional: ~/.local/bin/vacuum_gui + menu entry
```

`ros2 run` is the way to start it: ROS 2 installs an ament_python package's entry points in
`install/vacuum_gui/lib/vacuum_gui/` and does not add that directory to `PATH`. The desktop
installer writes `~/.local/bin/vacuum_gui`, a wrapper that sources ROS and the workspace and
then runs `ros2 run vacuum_gui vacuum_gui "$@"`, so the bare name works in a login shell.

| Panel | What it shows / does |
|---|---|
| **Stack** | every launch option with defaults remembered between runs: mode (AMCL on a saved map / SLAM), map, world, spawn pose, odometry source, headless, camera, dirt (count, seed, scale, stubborn share, brush radius, clean, randomise), Nav2 delay, and the live switches (gamepad, detector, recording, mission + meter, re-clean source, autostart, RViz). The exact command line is shown with a copy button, pre-flight checks are listed with fix hints, and `Kill leftovers` cleans up after a crashed run. Launch-time options are greyed out while the stack runs; changing one raises a `restart needed` badge with one-click `Apply & restart`. |
| **Map** | `/map` occupancy grid, `/coverage_map` cleaned area, `/scan` overlay, global `/plan` and `/local_plan`, `/coverage/plan_markers`, detected dirt from `/dirt/detections` with class colour and confidence labels, ground-truth dirt from `/dirt/markers`, AMCL particles, both costmaps, robot trail. Layer toggles in a side list; drag to pan, wheel to zoom, `Ctrl+M` to fit. A click is a Nav2 goal, a 2D pose estimate or "inspect this dirt spot", chosen in a selector. |
| **Camera** | `/camera/image` and `/dirt/debug_image` (boxes), each with its real frame rate and a `no signal` state instead of a frozen picture. |
| **Health** | a light per node (`/controller_server`, `/bt_navigator`, `/amcl` or `/slam_toolbox`, `/coverage_mission`, `/dirt_detector`, `/dirt_cleaner`, `/ros_gz_bridge`, …), a light per key topic (`/clock`, `/scan`, `/odom`, `/tf`, `/camera/image`, `/dirt/detections`) and one for the TF chain. Click a red light for the reason and the fix. |
| **Mission** | plan (preview), start, stop, reset meter, the coverage progress bar (reachable and all-free numbers labelled), the per-piece list from `/coverage/plan_markers`, the re-clean switch, map save/load. |
| **Robot** | hold-to-drive pad with linear/angular limits under the Nav2 caps, the gamepad indicator driven by `/joy`, and the emergency stop: it cancels the Nav2 goal and the mission and keeps publishing zero `/cmd_vel` at 10 Hz until released, because gz DiffDrive re-applies the last command. |
| **Dataset** | start/stop the labelled-frame recorder as its own process, count episodes and images on disk, and run `tools/dirt_dataset/package_dataset.py` to build the zip. |
| **Runs** | browse `~/vacuum_runs/run_*/`, read `summary.json`, compare two runs (coverage up, time/distance/missed area down) and plot `coverage.csv` over time. |
| **Experiments** | seeds × {no re-clean, map re-clean, vision re-clean}, headless, one at a time, with an on-screen table and `results.csv`. `false_recleans` counts clean-up spots that removed no patch during that pass (an approximation: pass end times from the run report, patch removal times from the dirt summary). |
| **Tools** | live `conf`/`iou`/`max_range`/`min_hits`/`clear_misses` on `/dirt_detector` via `ros2 param set`, screenshots and GIFs of the map or camera into `~/vacuum_runs/screenshots`, rosbag record toggle. |
| **Log** | one tab per managed process plus `All`, levels coloured, filter, save to file. |
| **Help** | panel guide and shortcuts (`F1`). |

**What is launch-time and what is live.** `vacuum.launch.py` fixes the world, the map, the
camera in the URDF, the dirt layout, the odometry source and the Nav2 delay. Everything else
is a separate process the panel owns and can stop without touching Gazebo:

| Live switch | process |
|---|---|
| gamepad | `ros2 launch vacuum_teleop teleop_joy.launch.py` |
| detector | `ros2 launch vacuum_vision detector.launch.py` |
| mission + meter | `ros2 launch vacuum_coverage coverage.launch.py` (generated params file when re-clean is off) |
| recording | `ros2 run vacuum_gazebo dirt_dataset.py` |
| RViz | `ros2 run rviz2 rviz2 -d <navigation.rviz>` |

That is why the preview shows `teleop:=none coverage:=false meter:=false detector:=false
collect_dataset:=false rviz:=false`: those parts are started afterwards by the panel. The
camera is the exception (it is part of the URDF), so detector and recording only work when
the launch had `cameras:=true`; the panel says so instead of failing silently.

**Threading.** rclpy spins in a `MultiThreadedExecutor` on its own thread; callbacks store the
latest message and emit Qt signals, and commands from the GUI go back through a queue drained
by a node timer, so no rclpy call ever runs on the GUI thread. Images are downscaled to
≤480 px and rate-limited (10 fps), the map redraws at 2 fps, scans at 5 fps. All ROS access
sits behind `vacuum_gui/backend.py`, so a real-robot backend (STM32 + RPLidar) can replace the
simulation backend without touching a panel.

**Shutdown.** Every child runs in its own session; stopping signals the whole process group
(SIGINT, then SIGTERM, then SIGKILL), which is what keeps `gz sim`, its ruby plugin host and
`rviz2` from surviving a stop. Closing the window while the stack runs asks whether to stop
it or leave it running.

Settings, presets and the dock layout live in `~/.config/vacuum_gui/`; runs, dirt truth,
datasets, bags and screenshots under `~/vacuum_runs/`. Nothing is written into the repository.

```bash
cd src/vacuum_gui && python3 -m pytest test -q     # 64 tests, no ROS and no display
```

## `vacuum_gui`: status, and what is left to do

**Verified on this machine** (Ubuntu 24.04, ROS 2 Jazzy, no display attached, `QT_QPA_PLATFORM=offscreen`):

- the package builds (`colcon build --symlink-install`) and `ros2 pkg executables vacuum_gui`
  lists `vacuum_gui` and `vacuum_gui_install_desktop`;
- the 64 ROS-free tests pass (command lines, settings/presets, occupancy-grid → RGBA and
  world↔pixel, scan/path maths, rate/RTF/localisation/health, process groups and orphan
  escalation with real child processes, run-report parsing and comparison, the experiment
  matrix + CSV + runner state machine against a fake `ros2`);
- the window renders offscreen with synthetic map, LiDAR, plans, dirt, particles, camera,
  dashboard and health;
- against a real headless stack (`vacuum_bringup vacuum.launch.py headless:=true`) the panel
  received the 99×98 occupancy grid, `map -> base_footprint` pose, `/scan`, `/clock` with a
  ~0.94 real-time factor, Nav2 lifecycle states and a fully green health panel.

**Not verified yet - check these on the machine with a GPU/display:**

1. **Start / Stop from the panel.** The last end-to-end attempt never launched the sim because
   a pre-flight modal was open in the test harness; the launch that the panel builds is the one
   verified above, but the button path itself needs one run. If Start does nothing, the `sim`
   tab in **Log** has the reason.
2. **Mission services.** Press *Plan* then *Start cleaning* with the coverage switch on; expect
   `/coverage/plan_markers` to draw the wall loops and lanes and `/coverage/status` to narrate.
   *Stop* and *Reset meter* should answer through the same path.
3. **A Nav2 goal from the map.** Click in "Nav2 goal" mode; the robot must drive, `/plan` must
   draw, and the action state should go `navigating → succeeded`.
4. **Camera panels.** Start with the camera on (`cameras:=true`) and check `/camera/image` and
   `/dirt/debug_image` frame rates; the boxes only appear while the detector is running and
   publishing.
5. **Detector tuning.** Change `conf` in **Tools** and confirm with
   `ros2 param get /dirt_detector conf`.
6. **Recording, packaging, rosbag, GIF.** Start recording during a mission, check the episode
   folder grows, then package it. The GIF export needs Pillow (`python3-pil`).
7. **E-stop.** While a mission runs, press `Space`: the mission must stop and `/cmd_vel` must
   stay zero until released (`ros2 topic echo /cmd_vel`).
8. **Wayland.** The panel works on native Wayland in principle; `--xcb` (or
   `ui.qt_platform: "xcb"` in `settings.json`) is the fallback if the Qt Wayland plugin
   misbehaves. The Gazebo and RViz windows the panel launches already go through XWayland.

**Known issues / punch list**

- A forced stop can leave one `ros_gz_bridge` behind (seen once). A leftover bridge publishes a
  **second `/clock`**, and that breaks AMCL (`Detected jump back in time`, no `map -> odom`)
  for the next run. `Kill leftovers` exists for exactly this; the stop path should also reap
  the bridge group, and the pre-flight check now lists bridges and Nav2 leftovers.
- `false_recleans` in the experiment table is a proxy, not ground truth (see the Experiments
  row). A real measure needs per-pass dirt counts in `summary.json`.
- The Stack panel is scrollable but tight at 950 px window height; collapsing the dirt and
  startup groups by default would help.
- Health marks a node "optional" only when it is absent and the configuration does not need
  it; when a node dies mid-run the row goes red but nothing beeps - there is no notification.
- The map view draws marker arrays as points/lines; `ARROW` markers from third-party tools are
  drawn as circles at their origin rather than as arrows.

## Status

Sim runs end to end on ROS 2 Jazzy: gz-sim + `ros_gz_bridge` + SLAM/Nav2, driven
by keyboard or gamepad (`vacuum_teleop`, verified on an EvoFox One S over
Bluetooth). Maps are saved into `maps/` and can be loaded with `slam:=False`.

The coverage mission, meter and one-command bringup run in Gazebo. The re-clean pass and run
report are unit-tested; the coverage figures quoted above (96.7 % / 75.7 %) are from the offline
planner. Dirt generation and the brush model are in and tested. The labelled-dataset pipeline has
been run in simulation (10 episodes) and a first dirt detector trained on it (see above). The
detector is not yet connected to the robot: the re-clean pass still plans from the map, not from
what the camera sees.

## Roadmap

Roughly in order, with honest status.

1. **Evaluate the detector on real floor photos — next.** Sim-only scores say little about a real
   floor; this decides how much to trust the model.
2. **Vision-guided re-clean — next.** A detector node that runs `models/dirt_detector/*.onnx` on
   `/camera/image`, turns each box into a floor position (the labelling geometry run backwards),
   merges repeated sightings, and hands the spots to the existing re-clean planner.
3. **Blind vs vision comparison — planned.** Same dirt seeds run three ways (no re-clean,
   map-based re-clean, vision-based re-clean): dirt left, extra time and distance, false re-cleans.
4. **Stop on signal loss — planned.** gz's DiffDrive keeps applying the last
   `/cmd_vel`, so a dropped Bluetooth link leaves the robot driving. Plan:
   `twist_mux` with the joystick on a high-priority input, Nav2 on a
   low-priority one, and a ~0.5 s timeout.
5. **Finish the map loop — partly done.** `save_map.sh` always writes to
   `<workspace>/maps`, and AMCL loads it with
   `slam:=False map:=<workspace>/maps/<name>.yaml`. Remaining:
   `navigation.launch.py` still defaults to `vacuum_gazebo/maps`.
6. **More environments — planned.** A room generator for new worlds and maps, so the detector and
   the coverage planner are not tuned to one living room.
7. **Own robot description — planned.** Replace `vacuum_description` with our
   own URDF, then retune the Nav2 footprint and inflation.
