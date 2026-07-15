# custom_joy_node

Custom ROS 2 joy node written in C++.

This node is designed to keep publishing safe `sensor_msgs/msg/Joy` messages even when no
controller is connected.

## Behavior

- Scans `/dev/input/js0` to `/dev/input/js31` automatically, then falls back to `/dev/input/event0` to `/dev/input/event63`.
- Publishes zero-filled `joy` messages when no controller is connected.
- Detects controller disconnection, logs a warning, and keeps publishing zeros.
- Reconnects automatically when a controller appears again.
- Publishes axes/buttons exposed by Linux input devices.
- Prefers Linux joystick devices (`/dev/input/js*`) so runtime input matches the Joy Profile Editor by default.

## Build

```bash
colcon build --packages-select custom_joy_node
```

## Run

```bash
ros2 run custom_joy_node custom_joy_node
```

or with launch:

```bash
ros2 launch custom_joy_node custom_joy_node.launch.py joy_topic:=/joy
```

## Parameters

```bash
ros2 run custom_joy_node custom_joy_node --ros-args \
  -p device_path:=/dev/input/js0 \
  -p publish_rate_hz:=50.0 \
  -p scan_period_ms:=1000 \
  -p default_axes_count:=8 \
  -p default_buttons_count:=16 \
  -p deadzone:=0.05 \
  -p prefer_evdev:=false
```

- Topic is fixed to `/joy`; change it from launch remapping with `joy_topic:=...`.
- `device_path`: set this to a fixed device such as `/dev/input/event5` or `/dev/input/js0`; leave empty to scan automatically.
- `default_axes_count` / `default_buttons_count`: output size while no controller is connected.
- `deadzone`: values smaller than this are published as zero.
- `prefer_evdev`: use `/dev/input/event*` before `/dev/input/js*`; default is false.

The same parameters can be edited in `config/custom_joy_node.param.yaml` or overridden from launch:

```bash
ros2 launch custom_joy_node custom_joy_node.launch.py \
  joy_topic:=/vehicle/joy \
  device_path:=/dev/input/event5 \
  deadzone:=0.05
```

## DualShock 4 L2/R2

On many Linux setups, DualShock 4 L2/R2 are available as analog evdev axes:

- `ABS_Z`: L2
- `ABS_RZ`: R2

With `prefer_evdev:=true`, this node publishes those trigger values as axes in the `0.0` to `1.0`
range when the kernel driver exposes them. If the device only exposes L2/R2 as buttons, the node
can only publish them as `0` / `1`; in that case check the connection mode, driver, and permissions
for `/dev/input/event*`.
