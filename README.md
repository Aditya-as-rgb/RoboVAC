# vacuum_sim

ROS 2 **Jazzy** + Gazebo (gz-sim) simulation of a round robot vacuum: SLAM, Nav2,
teleop, bumpers. Runs natively on Ubuntu 24.04, no Docker, no `kaiaai` packages.

The robot model, worlds and Nav2 tuning are derived from the open-source
[OOMWOO](https://github.com/makerspet/oomwoo) project by Maker's Pet / KAIA.AI
(Apache-2.0). See [NOTICE](NOTICE). The plan is to start with the OOMWOO One URDF
unchanged and later replace it with our own design.

## Layout

| Package | Contents |
|---|---|
| `vacuum_description` | `urdf/` xacro (body, wheels, LiDAR, ToF, side ranges, IMU, bumpers, gz plugins), `config/gz_bridge.yaml` (gz <-> ROS topic bridge), `config/ekf.yaml` (for a real robot, unused in sim), RViz configs |
| `vacuum_gazebo` | `worlds/living_room.world`, `models/`, `maps/`, `launch/sim.launch.py` |
| `vacuum_bringup` | `launch/vacuum.launch.py` (everything in one command), `launch/navigation.launch.py` (SLAM or AMCL + Nav2 + RViz), `config/navigation.yaml`, `config/explore_lite.yaml` (for later) |
| `vacuum_coverage` | wall-pass + lane-sweep coverage mission on top of Nav2 (`coverage_mission`), cleaned-area overlay and percentage (`coverage_meter`), offline planner/preview (`coverage_preview`), unit tests |
| `vacuum_teleop` | `launch/teleop_joy.launch.py` + `config/teleop_joy.yaml` — gamepad teleop (`joy` + `teleop_twist_joy` -> `/cmd_vel`) |

## Install dependencies

```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher python3-colcon-common-extensions \
  python3-rosdep python3-numpy python3-yaml python3-matplotlib python3-pytest
```

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
Gazebo gets before Nav2 starts, default 8). Without `autostart`, start or stop a clean from
another terminal:

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
(if the Gazebo window crashes on OpenGL), `enable_tof:=false enable_cameras:=false`
(faster), `odom_source:=robot_wheels` (wheel odometry that drifts, instead of
ground truth), `x_pose`, `y_pose`, `world`.

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
python3 -m pytest test        # planner, controller model and mission logic tests
```

Tuning: `src/vacuum_coverage/config/coverage.yaml`. Keep `pursuit_lookahead`/`pursuit_speed`
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
| TF chain | `map -> odom -> base_footprint -> base_link -> base_scan` (+ wheel, sensor links) |
| Topics out | `/scan` (frame `base_scan`), `/odom`, `/imu`, `/tf`, `/joint_states`, `/clock`, `/bumper_left/contact`, `/bumper_right/contact` |
| Topic in | `/cmd_vel` (`geometry_msgs/Twist`), a diff-drive plugin |
| gz model name | `vacuum` (set by `sim.launch.py`, referenced in `config/gz_bridge.yaml`) |
| gz world name | `default` (also referenced in `gz_bridge.yaml`) |
| Footprint | Nav2 radius/footprint in `navigation.yaml` is tuned for ~0.349 m diameter; update it when your body size changes |

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

## Status

Sim runs end to end on ROS 2 Jazzy: gz-sim + `ros_gz_bridge` + SLAM/Nav2, driven
by keyboard or gamepad (`vacuum_teleop`, verified on an EvoFox One S over
Bluetooth). Maps are saved into `maps/` and can be loaded with `slam:=False`.

Coverage mission, meter and one-command bringup are written and unit-tested offline
(planner, controller model, mission sequencing) but have not been run against Gazebo yet.

## Roadmap

Roughly in order, with honest status.

1. **Stop on signal loss — planned.** gz's DiffDrive keeps applying the last
   `/cmd_vel`, so a dropped Bluetooth link leaves the robot driving. Plan:
   `twist_mux` with the joystick on a high-priority input, Nav2 on a
   low-priority one, and a ~0.5 s timeout.
2. **Finish the map loop — partly done.** `save_map.sh` always writes to
   `<workspace>/maps`, and AMCL loads it with
   `slam:=False map:=<workspace>/maps/<name>.yaml`. Remaining:
   `navigation.launch.py` still defaults to `vacuum_gazebo/maps`.
3. **One-command bringup — written, untested in sim.** `vacuum.launch.py`, see above.
4. **Cleaning/coverage behaviour — written, untested in sim.** Wall pass + lane sweep
   with a coverage meter (`vacuum_coverage`). Next: run it on the living room, tune
   lane spacing, then try it with the own-robot URDF.
5. **Own robot description — planned.** Replace `vacuum_description` with our
   own URDF, then retune the Nav2 footprint and inflation.

