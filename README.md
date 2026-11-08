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
| `vacuum_bringup` | `launch/navigation.launch.py` (SLAM or AMCL + Nav2 + RViz), `config/navigation.yaml`, `config/explore_lite.yaml` (for later) |
| `vacuum_teleop` | `launch/teleop_joy.launch.py` + `config/teleop_joy.yaml` — gamepad teleop (`joy` + `teleop_twist_joy` -> `/cmd_vel`) |

## Install dependencies

```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher python3-colcon-common-extensions \
  python3-rosdep
```

## Build

```bash
cd ~/vacuum_sim
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run (one terminal each, source `install/setup.bash` in every one)

```bash
# 1. Gazebo + robot (living room)
ros2 launch vacuum_gazebo sim.launch.py

# 2. SLAM + Nav2 + RViz: map while you drive / send goals
ros2 launch vacuum_bringup navigation.launch.py

# 3. Drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# ...or with a gamepad (see "Gamepad teleop" below)
ros2 launch vacuum_teleop teleop_joy.launch.py

# 4. Save the map when done (always into ~/vacuum_sim/maps)
ros2 run vacuum_bringup save_map.sh living_room
```

Navigate on a saved map instead of mapping (step 2 then becomes this):

```bash
ros2 launch vacuum_bringup navigation.launch.py slam:=False \
  map:=$HOME/vacuum_sim/maps/living_room.yaml
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

Not implemented yet: the cleaning/coverage behaviour (the upstream coverage
packages depend on other OOMWOO packages and are not vendored here).

## Roadmap

Roughly in order, with honest status.

1. **Stop on signal loss — planned.** gz's DiffDrive keeps applying the last
   `/cmd_vel`, so a dropped Bluetooth link leaves the robot driving. Plan:
   `twist_mux` with the joystick on a high-priority input, Nav2 on a
   low-priority one, and a ~0.5 s timeout.
2. **Finish the map loop — partly done.** `save_map.sh` always writes to
   `<workspace>/maps`, and AMCL loads it with
   `slam:=False map:=$HOME/vacuum_sim/maps/<name>.yaml`. Remaining:
   `navigation.launch.py` still defaults to `vacuum_gazebo/maps`.
3. **One-command bringup — planned.** A single launch file starting sim +
   navigation + teleop, with `slam`, `map`, `rviz` and `headless` arguments.
4. **Cleaning/coverage behaviour — planned.** Vendor the upstream OOMWOO
   coverage packages or implement a boustrophedon coverage node on top of Nav2.
5. **Own robot description — planned.** Replace `vacuum_description` with our
   own URDF, then retune the Nav2 footprint and inflation.

