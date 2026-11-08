# vacuum_teleop

Gamepad teleop for the vacuum simulation. Thin wrapper around
[`joy`](https://index.ros.org/p/joy/) and
[`teleop_twist_joy`](https://index.ros.org/p/teleop_twist_joy/), both stock
ROS 2 Jazzy packages — no custom node code.

```bash
ros2 launch vacuum_teleop teleop_joy.launch.py
```

Publishes `geometry_msgs/msg/Twist` on `/cmd_vel`, which `gz_bridge.yaml` in
`vacuum_description` forwards to the gz-sim DiffDrive plugin.

## Controls

| Input | Action |
|---|---|
| **LB** (hold) | deadman — no motion unless held |
| **Left stick ↑ / ↓** | forward / reverse, ±0.30 m/s |
| **Left stick ← / →** | turn left / right, ±1.20 rad/s |
| **RB** (hold) | turbo, ±0.50 m/s and ±2.50 rad/s |

Scales live in `config/teleop_joy.yaml` and are capped at the Nav2 limits in
`vacuum_bringup/config/navigation.yaml`.

## Arguments

| Argument | Default | Meaning |
|---|---|---|
| `joy_dev` | `0` | SDL joystick index (`ros2 run joy joy_enumerate_devices`) |
| `joy_vel` | `cmd_vel` | Twist topic to publish |
| `config` | `config/teleop_joy.yaml` | parameter file |

## Notes

- The pad must exist as `/dev/input/js0` (kernel `xpad` driver) before launch.
  With a Bluetooth controller that means a real bond, not just `Paired: yes` —
  see the BlueZ `Rejected connection from !bonded device` pitfall.
- `joy` 3.3.0 is SDL-based and names the device differently from the kernel
  ("Xbox One S Controller" vs "Xbox Wireless Controller"). Select by
  `device_id`, never by name.
- Nav2 also publishes `/cmd_vel`. Run one driver at a time; use `twist_mux`
  with a remapped `joy_vel` if both must coexist.
- Axis signs differ between pads. If forward and turn are inverted, flip the
  signs of `scale_linear.x` and `scale_angular.yaw`.
