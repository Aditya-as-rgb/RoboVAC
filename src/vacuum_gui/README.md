# vacuum_gui

Control panel for the RoboVAC simulation: one window to start and stop the whole stack,
see what the robot sees and knows, and drive it.

PySide6 (Qt 6). Gazebo and RViz are **not** embedded: the map, camera, LiDAR, plans and
dirt are drawn from ROS topics, so the simulation can run headless (about half the GPU load
of a Gazebo window) and RViz is optional. The buttons for the real Gazebo and RViz windows
are still there.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ Start stack  Stop stack  [E-STOP]  Plan  Start cleaning  Stop cleaning  …     │
├───────────────┬──────────────────────────────────┬───────────────────────────┤
│ Stack         │ Map                              │ Camera / Mission / Robot  │
│ (every launch │  occupancy grid + cleaned area   │ / Dataset (tabs)          │
│  option, the  │  + LiDAR + plans + dirt +        ├───────────────────────────┤
│  exact        │  particles + costmaps            │ Health                    │
│  command,     │                                  │                           │
│  pre-flight)  │  click: goal / pose / inspect     │                           │
├───────────────┴──────────────────────────────────┴───────────────────────────┤
│ Log / Runs / Experiments / Tools (tabs)                                       │
├──────────────────────────────────────────────────────────────────────────────┤
│ sim time · RTF · coverage (reachable / all free) · mission · dirt · speed …   │
└──────────────────────────────────────────────────────────────────────────────┘
```

## Install

PySide6 is the only new dependency. On Ubuntu 24.04 the apt package name is not available
on every mirror, so the tested route is the user install into the system Python (the one
`rclpy` uses):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages PySide6
```

The simulation's own dependencies are unchanged (`onnxruntime` and `numpy<2` for the
detector, `python3-numpy`, `python3-yaml`, `python3-pil` for this package). Qt 6.12 needs
glibc 2.34 or newer, which 24.04 has.

## Build and run

```bash
cd ~/vacuum_sim
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select vacuum_gui
source install/setup.zsh
ros2 run vacuum_gui vacuum_gui
```

ROS 2 puts an ament_python package's entry points in `install/<pkg>/lib/<pkg>/` and does
not add that directory to `PATH`, so `vacuum_gui` on its own is not a command after
sourcing; `ros2 run vacuum_gui vacuum_gui` is. If you want the bare name, run the desktop
installer below once - it writes `~/.local/bin/vacuum_gui`, which is on `PATH` in a normal
login shell and sources ROS and the workspace itself.

`ros2 launch vacuum_gui gui.launch.py` starts the same panel (the launch file starts only
the panel - the panel starts the simulation itself, so there is never a second Gazebo).
`--xcb` forces Qt through XWayland if the native Wayland plugin misbehaves;
`QT_QPA_PLATFORM` set in the environment always wins.

Desktop launcher and the bare command (no sudo, writes only into `~/.local`):

```bash
ros2 run vacuum_gui vacuum_gui_install_desktop
```

It creates `~/.local/bin/vacuum_gui` (a wrapper that sources ROS + `install/setup.zsh`
and runs `ros2 run vacuum_gui vacuum_gui "$@"`), a `vacuum_gui_launch` alias,
`~/.local/share/applications/vacuum_gui.desktop` and the icon. Remove it with
`ros2 run vacuum_gui vacuum_gui_install_desktop --uninstall`.

## The panels

- **Stack** - mode (localise on a saved map / map while driving), map, world, spawn pose,
  odometry source, headless, camera, dirt (count, seed, scale, stubborn share, brush,
  clean, randomise), Nav2 delay, and the live switches (gamepad, detector, recording,
  mission + meter, re-clean and its source, autostart, RViz). It shows the exact command
  line that will run and a copy button, the pre-flight result, and `Kill leftovers`.
  Launch-time options are greyed out while the stack runs; changing one raises a
  `restart needed` badge with `Apply & restart`.
- **Map** - `/map` occupancy grid, `/coverage_map` cleaned area, `/scan` overlay, global
  (`/plan`) and local (`/local_plan`) plans, `/coverage/plan_markers`, detected dirt from
  `/dirt/detections` with class colour and confidence labels, ground-truth dirt from
  `/dirt/markers`, AMCL particles, and both costmaps. Layer toggles sit in the side list;
  drag to pan, wheel to zoom, Ctrl+M to fit. A click means a Nav2 goal, a 2D pose estimate
  or "inspect this dirt spot", chosen in the click-mode selector.
- **Camera** - `/camera/image` and `/dirt/debug_image` (boxes), each with its own frame
  rate and a `no signal` state instead of a frozen picture.
- **Health** - a light per node (`/controller_server`, `/bt_navigator`, `/amcl` or
  `/slam_toolbox`, `/coverage_mission`, `/dirt_detector`, `/dirt_cleaner`,
  `/parameter_bridge`, …), a light per key topic (`/clock`, `/scan`, `/odom`, `/tf`,
  `/camera/image`, `/dirt/detections`) and one for the TF chain. Double-click a red light
  for the reason and the fix.
- **Mission** - plan (preview), start, stop, reset meter, the coverage progress bar
  (reachable and all-free numbers), the per-piece list built from `/coverage/plan_markers`,
  the re-clean switch, and map save/load.
- **Robot** - hold-to-drive pad with linear/angular limits (defaults under the Nav2 caps),
  the gamepad indicator driven by `/joy`, and the emergency stop, which cancels the Nav2
  goal and the mission and keeps publishing zero `/cmd_vel` ten times a second until it is
  released (gz DiffDrive keeps the last command, so one zero is not enough).
- **Dataset** - start/stop the labelled-frame recorder as its own process, count episodes
  and images on disk, and run `tools/dirt_dataset/package_dataset.py` to build the zip.
- **Runs** - browse `~/vacuum_runs/run_*/`, read `summary.json`, compare two runs (coverage
  up, time/distance/missed area down), and plot `coverage.csv` over time.
- **Experiments** - seeds × {no re-clean, map re-clean, vision re-clean}, headless, one at a
  time; writes `results.csv` and the rows are visible as they finish. `false_recleans` is
  an approximation: clean-up spots in a pass that removed no patch during that pass (dirt
  removal times come from the dirt summary, pass end times from the run report).
- **Tools** - live `conf`, `iou`, `max_range`, `min_hits`, `clear_misses` on
  `/dirt_detector` via `ros2 param set`, screenshots and GIFs of the map or camera into
  `~/vacuum_runs/screenshots`, and a rosbag record toggle.
- **Log** - one tab per managed process plus `All`, levels coloured, filter box, save.
- **Help** - the same panel guide and the shortcut list (F1).

Shortcuts: `F5` start/stop the stack, `Space` e-stop, `Ctrl+P` plan, `Ctrl+Return` start
cleaning, `Ctrl+.` stop cleaning / cancel goal, `Ctrl+M` fit map, `Ctrl+Shift+K` kill
leftovers, `F1` help.

## What is launch-time and what is live

`vacuum.launch.py` is one `ros2 launch` tree: the world, the map, the camera in the URDF,
the dirt layout, the odometry source and the Nav2 start delay are fixed when it starts.
Everything else is **a separate process the panel owns**, so it can be switched without
touching Gazebo:

| Live switch | runs | stop |
|---|---|---|
| gamepad | `ros2 launch vacuum_teleop teleop_joy.launch.py` | SIGINT the group |
| detector | `ros2 launch vacuum_vision detector.launch.py` | SIGINT |
| mission + meter | `ros2 launch vacuum_coverage coverage.launch.py` (with a generated params file when re-clean is off) | SIGINT |
| recording | `ros2 run vacuum_gazebo dirt_dataset.py` | SIGINT |
| RViz | `ros2 run rviz2 rviz2 -d …` | SIGINT |

That is why the command preview shows the sim launch with `teleop:=none coverage:=false
meter:=false detector:=false collect_dataset:=false rviz:=false`: those are started
afterwards by the panel. The camera is the one exception - it is in the URDF, so the
detector and recording switches only work when the launch had `cameras:=true`; the panel
says so instead of failing silently.

## Architecture

```
app.py         QApplication, theme, backend selection, window
theme.py       8 colour themes + System, QSS built from one palette
feedback.py    press flash + busy (spinner) state for buttons
toast.py       floating result cards (spinner -> tick/cross)
background.py  run a blocking call off the GUI thread
main_window.py docks, orchestration (start/stop/restart, live toggles, e-stop), shortcuts
backend.py     RobotBackend signals + NullBackend (the only interface panels know)
ros_backend.py rclpy node in its own thread, MultiThreadedExecutor, throttled signals
launch_manager.py  process groups (setsid), SIGINT->SIGTERM->SIGKILL, log bus, leftovers
commands.py    StackConfig, every argv the panel runs, child environment
preflight.py   checks with fix hints (ROS, packages, model, map, ports, leftovers, …)
settings.py    ~/.config/vacuum_gui/settings.json + presets.json (defaults merged)
paths.py       workspace, maps, models, runs discovery
gridmap.py     occupancy grid -> RGBA, world<->pixel, scan and path maths (numpy)
status.py      rate meter, RTF, localisation state, health lights
map_view.py    QGraphicsScene layers (map, coverage, scan, plans, markers, particles)
camera_view.py camera/debug views with FPS and no-signal state
*_panel.py     one panel each; they only emit intent and consume signals
run_report.py  read run_*/summary.json + coverage.csv, compare two runs
experiments.py experiment matrix, results table, sequential runner
```

ROS never runs on the GUI thread: the node spins in `MultiThreadedExecutor` in a Python
thread, callbacks store the latest message and emit a Qt signal (Qt queues it to the GUI),
and commands from the GUI go through a lock-protected queue drained by a 50 Hz node timer.
Images are downscaled to ≤480 px wide in the ROS thread and rate-limited (10 fps default);
the map is redrawn at 2 fps, scans at 5 fps.

All ROS-free logic is unit-tested without ROS and without a display:

```bash
cd src/vacuum_gui
python3 -m pytest test -q
```

Covered: command-line building (including the bool spellings the launch files declare),
settings/preset merge and save, occupancy-grid → RGBA and world↔pixel, scan/path maths,
rate/RTF/localisation/health logic, process groups and orphans (with real child
processes), run-report parsing and comparison, the experiment matrix, the results CSV and
the runner state machine (against a fake `ros2`).

## Settings and files

| Path | Contents |
|---|---|
| `~/.config/vacuum_gui/settings.json` | last Stack configuration, view toggles, theme |
| `~/.config/vacuum_gui/presets.json` | presets (five are built in, yours are added here) |
| `~/.config/vacuum_gui/layout.json` | dock layout and window geometry |
| `~/vacuum_runs/` | runs, dirt truth, datasets, bags, screenshots, experiments |
| `<workspace>/maps/` | saved maps (`save_map.sh`) |

Nothing is written into the repository.

## Honest status

Tested on this machine without a display: the ROS-free tests pass, the window renders
offscreen with synthetic data, and the panel has been run against a real headless stack -
the map, `map -> base_footprint`, `/scan`, `/clock`, Nav2 lifecycle states and the health
lights all arrived, Start launched the tree and Stop left no `gz`/bridge/Nav2 process
behind. The services (plan/start/stop/reset), the `NavigateToPose` action, the detector
parameter client and the camera-facing panels were **not exercised against a running
Gazebo here**; the checklist for those is in the repository README under
`vacuum_gui: status, and what is left to do`.

`python3 -m pytest test -q` runs 66 tests.

## Themes and button feedback

* Pick a theme from the **Theme** drop-down at the right of the toolbar (System, Graphite,
  Midnight, Nord, Dracula, Mocha, High contrast, Daylight, Solarized light). It applies at
  once and is remembered. `vacuum_gui --theme nord` overrides it for one run.
* Every button flashes when clicked. Slow actions (Start/Stop stack, Plan, Start cleaning,
  Save map...) turn the button into a spinner with a verb, show a toast card, and finish with a
  green or red ring plus a toast with the outcome - no need to watch the terminal.
* Settings: `ui.preflight_warnings=false` (or the "don't ask again" box) skips the
  warnings-only pre-flight dialog.
