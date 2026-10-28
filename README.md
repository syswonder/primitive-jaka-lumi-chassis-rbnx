# primitive-jaka-lumi-chassis-rbnx

Robonix package for the JAKA Lumi differential-drive chassis. It adapts the
vendor's **JSON-over-TCP** interface (`192.168.10.10:31001`) to the standard
`robonix/primitive/chassis/*` capabilities.

```text
lumi_chassis/
├── tcp_client.py   vendor string protocol: one TCP connection and one JSON reply per command
├── driver.py       config validation + odometry publishing + velocity refresh + closed-loop forward/rotate
└── main.py         Primitive lifecycle and gRPC entry point
```

## Capabilities

| Contract | Mode | Transport | Endpoint |
|---|---|---|---|
| `robonix/primitive/chassis/odom` | `topic_out` | ros2 | `/odom` |
| `robonix/primitive/chassis/twist_in` | `topic_in` | ros2 | `/cmd_vel` |
| `robonix/primitive/chassis/move` | `rpc` | grpc | `ExecuteMoveCommand` |

Frames, units, `move` semantics and error semantics are in
[CAPABILITY.md](CAPABILITY.md); the configuration fields are in
[config.spec](config.spec).

## Prerequisites

- Robonix installed with the source root registered: `rbnx setup ~/robonix`
- ROS 2 Humble (`/opt/ros/humble`) with `nav_msgs`, `tf2_msgs` and `colcon`
- A route to the chassis. A WiFi connection needs an extra static
  `192.168.10.x/24` address
  (`nmcli connection modify Lumi10000023 +ipv4.addresses 192.168.10.200/24`);
  the `10.5.5.x` pool belongs to the AP itself and cannot reach the base.

## Build

```bash
bash scripts/build.sh
# equivalent to: venv + rbnx codegen --mcp --ros2 + colcon build (the ros2_idl overlay)
```

Manifest check only, no build:

```bash
rbnx validate .
```

## Standalone start (debugging)

```bash
rbnx start -p . --endpoint 127.0.0.1:50051 --set motion_enabled=false
```

`motion_enabled` defaults to `false`: only `/odom` is published and every motion
command is rejected with a warning. Enable it once the odometry and the e-stop
state look right:

```bash
rbnx start -p . --set motion_enabled=true --set max_linear_m_s=0.1
```

## Verification

```bash
# 1) provider and capabilities show up in Atlas
rbnx caps -v | rg -A6 lumi_chassis

# 2) odometry is moving (needs the base to actually move, or drive it with move)
ros2 topic echo /odom --once

# 3) is the transform chain connected
ros2 run tf2_ros tf2_echo odom base_link

# 4) single-shot command (the robot really moves — clear the area first)
#    via rbnx ask or any gRPC client: chassis/move with forward_m=0.1
```

## Known limitations

- **No wheel encoders.** `/odom` publishes the vendor SLAM pose, not integrated
  wheel speed. Mapping corrects the drift through `map -> odom`; `twist` is a
  finite difference of that pose and only approximate.
- **No obstacle avoidance.** `/cmd_vel` goes to `/api/joy_control`, and the base
  does not use its own navigation avoidance in that mode. For avoidance drive
  the vendor path with `move` (`/api/move`), or give Nav2 a scan provider on the
  Robonix side.
- **Single instance.** One process drives one chassis; multiple instances need
  different `host`/`port` values.
- The vendor API has no version negotiation. Field changes surface through the
  error table in `CAPABILITY.md`.

## License

Apache-2.0.
