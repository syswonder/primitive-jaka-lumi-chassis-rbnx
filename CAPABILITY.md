---
description: Drive the JAKA Lumi differential-drive base — read the vendor-SLAM pose, send bounded forward/rotate moves, or stream /cmd_vel. Motion is gated off by default.
---

# lumi_chassis capability notes

Provider ID is the deployment instance `name` (default `lumi_chassis`). All
three capabilities come from the canonical tree in
`<robonix>/capabilities/primitive/chassis/` — this package defines no local
contracts.

## Frames and units

- **`odom`** (`topic_out`) carries `nav_msgs/Odometry` every
  `odom_rate_hz` (default 10 Hz). `header.frame_id` = `config.odom_frame`
  (default `odom`), `child_frame_id` = `config.base_frame` (default
  `base_link`). The provider also publishes the matching `odom → base_link`
  TF when `publish_tf` is true.
- The Lumi base exposes **no wheel encoders**. This "odometry" is the vendor
  SLAM pose, i.e. a map-frame estimate served as if it were odom. Mapping
  removes the residual drift through `map → odom`; do not expect
  encoder-grade twist.
- `config.vendor_pose_offset` applies a rigid 2-D transform
  `[x, y, yaw]` to that pose. Leave it at zero unless you deliberately align
  the vendor map with a Robonix map. **Two unaligned maps silently send the
  robot to the wrong place** — pick one navigation owner.
- **`twist_in`** (`topic_in`) consumes `geometry_msgs/Twist` on `/cmd_vel`.
  Only `linear.x` (m/s) and `angular.z` (rad/s) are used; every other field
  is ignored. Linear and angular values are clamped to
  `max_linear_m_s` / `max_angular_rad_s`.
- **`move`** (`rpc`, gRPC) takes `chassis/ExecuteMoveCommand` and returns a
  `std_msgs/String` whose `data` is a JSON object:
  `{"status": "<accepted|rejected|failed|timeout|aborted>", "detail": "..."}`.

## Call order

1. `CMD_INIT` validates the config, declares the ROS 2 endpoints and performs
   one `/api/robot_status` round-trip. An unreachable base fails init with the
   endpoint in the message (this is a hard dependency, not `Deferred`).
2. `CMD_ACTIVATE` starts the odom publisher and the velocity refresher.
   Repeating it is idempotent.
3. Call `move` or publish `/cmd_vel` (`motion_enabled: true` required).
4. `CMD_DEACTIVATE` / `CMD_SHUTDOWN` stop sending, command zero velocity up to
   three times and join the threads.

## `move` semantics

Priority: `forward_m` > `rotate_deg` > velocity fields.

| Field | Behaviour |
|---|---|
| `forward_m != 0` | Closed-loop straight line along the heading sampled at call time. Speed `clamp(|forward_m| / 2, 0.03, max_linear_m_s)`. Arrival tolerance `move_tolerance_m`. |
| `rotate_deg != 0` | Closed-loop in-place yaw. Speed `clamp(|angle| / 2, 10 deg/s, max_angular_rad_s)`. Tolerance `rotate_tolerance_deg`. |
| velocity fields | Linear.x / angular.z held for `duration_sec`, or `default_move_duration_s` when 0. Non-planar fields (`linear.y/z`, `angular.x/y`) are rejected. |

A move is refused outright when:

- `motion_enabled` is false;
- the chassis reports an engaged e-stop (`estop_state`, `hard_estop_state` or
  `soft_estop_state` truthy);
- a vendor navigation task is running (`move_status == "running"`) — cancel it
  first.

## Stop behaviour

The vendor `joy_control` command stays valid for only **0.5 s**, so the
provider re-sends it at `joy_rate_hz` (default 5 Hz, minimum 2 Hz). A
`/cmd_vel` command expires after `twist_timeout_s` (default 0.5 s); if no new
message arrives, the provider sends a stop. Combined with the shutdown stop
retries this is what prevents a crashed consumer from leaving the base
driving.

## Typical errors

| Symptom | Cause |
|---|---|
| `move` returns `rejected: motion_enabled is false` | safety gate still on |
| `move` returns `rejected: a vendor navigation task is already running` | `/api/move` was used (vendor stack); cancel it or use that stack only |
| init fails with `chassis unreachable` | missing `192.168.10.x/24` static address on the WiFi link, or the base is off |
| `/odom` stops while `/tf` continues (or vice versa) | transient TCP error; the provider drops the pose after 3 consecutive failures instead of publishing a stale one |
