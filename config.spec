# Runtime config accepted by the JAKA Lumi chassis primitive.
#
# This file documents the `config:` mapping of this package's deployment
# entry in robonix_manifest.yaml. It is documentation for deployers and
# tooling; the provider re-validates every value in on_init and returns a
# located error instead of silently clamping.

config:
  # string, default: 192.168.10.10.
  # Chassis controller address. This is the Lumi base board, NOT the arm
  # cabinet (192.168.10.90) and not the WiFi AP pool (10.5.5.x).
  host: 192.168.10.10

  # int, 1..65535, default: 31001.
  # TCP port of the vendor string-command API (one JSON reply per command).
  port: 31001

  # float, seconds, default: 3.0; must be > 0.
  # Per-command socket timeout. The vendor answers in <100 ms on a healthy
  # link, so a timeout means the base is unreachable.
  cmd_timeout_s: 3.0

  # bool, default: false. SAFETY GATE.
  # When false the provider only publishes odom/TF and rejects every motion
  # command (move RPC and twist_in) with a rate-limited warning. Follow the
  # staged acceptance flow: boot read-only, verify odom and estop, then set
  # this to true.
  motion_enabled: false

  # string, default: odom.
  # Parent frame of nav_msgs/Odometry and of the odom -> base_frame TF.
  # The Lumi base exposes no wheel encoders, so this "odom" is the vendor
  # SLAM pose in the vendor map; Mapping corrects the residual drift through
  # map -> odom.
  odom_frame: odom

  # string, default: base_link.
  # Child frame of nav_msgs/Odometry and of the odom -> base_frame TF.
  # Must be the root link of the deployment URDF (soma.yaml urdf.root_link).
  base_frame: base_link

  # bool, default: true.
  # Publish the odom -> base_frame TF in addition to the odometry topic.
  # Disable only if another component already owns that transform.
  publish_tf: true

  # float, Hz, default: 10.0; must be >= 1.0.
  # Poll /api/robot_status and publish odom at this rate. The vendor status
  # call is a full round-trip TCP connection; do not raise above ~20 Hz.
  odom_rate_hz: 10.0

  # float, m/s, default: 0.3; must be > 0 and <= 0.5.
  # Linear speed cap for both move and twist_in. 0.5 is the chassis firmware
  # limit; values above it are rejected at init.
  max_linear_m_s: 0.3

  # float, rad/s, default: 0.5; must be > 0 and <= 1.0.
  # Angular speed cap. 1.0 is the chassis firmware limit.
  max_angular_rad_s: 0.5

  # float, Hz, default: 5.0; must be >= 2.0.
  # joy_control resend rate. Each vendor joy command stays valid for only
  # 0.5 s, so anything below 2 Hz produces stuttering motion.
  joy_rate_hz: 5.0

  # string, default: lumi.
  # Vendor `uuid` query field. Keeps two robots on one network apart.
  uuid: lumi

  # map, optional, default: {x: 0.0, y: 0.0, yaw: 0.0}.
  # Rigid 2-D alignment from the vendor map into the Robonix map frame:
  #   [x_r, y_r] = R(yaw) * [x_v, y_v] + [x, y],  theta_r = theta_v + yaw
  # Leave at zero unless you deliberately align the vendor SLAM map with a
  # Robonix (RTAB-Map) map. Mixing two unaligned maps silently drives the
  # robot to the wrong place.
  vendor_pose_offset: {x: 0.0, y: 0.0, yaw: 0.0}

  # float, seconds, default: 1.0; must be > 0.
  # Default hold time for the velocity branch of the move RPC when the
  # request leaves duration_sec at 0.
  default_move_duration_s: 1.0

  # float, seconds, default: 20.0; must be > 0.
  # Upper bound for a closed-loop forward_m / rotate_deg execution.
  move_timeout_s: 20.0

  # float, seconds, default: 0.5; must be > 0.
  # If no twist_in message arrives for this long, the last velocity command
  # expires and the provider sends a stop. This is what keeps a crashed
  # Nav2 from leaving the base driving.
  twist_timeout_s: 0.5

  # float, metres, default: 0.02; must be >= 0.
  # Closed-loop arrival tolerance for forward_m / rotate_deg.
  move_tolerance_m: 0.02

  # float, deg, default: 2.0; must be >= 0.
  # Closed-loop arrival tolerance for rotate_deg.
  rotate_tolerance_deg: 2.0

  # bool, default: true.
  # Send up to three stop commands on shutdown. Idempotent, so repeating it
  # is harmless.
  stop_on_shutdown: true
