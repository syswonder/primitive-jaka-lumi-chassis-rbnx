# SPDX-License-Identifier: Apache-2.0
"""JAKA Lumi chassis driver.

Adapts the vendor JSON-over-TCP API to the Robonix chassis contracts:

  odom      topic_out  nav_msgs/Odometry on /odom (+ optional odom -> base TF)
  twist_in  topic_in   geometry_msgs/Twist on /cmd_vel
  move      rpc        forward_m | rotate_deg | bounded velocity burst

Threading model
---------------
``_odom_loop``   polls /api/robot_status at ``odom_rate_hz``, caches the pose
                 and publishes odometry + TF.
``_ctrl_loop``   re-sends the currently requested velocity at ``joy_rate_hz``.
                 The vendor joy command stays valid for only 0.5 s, so it must
                 be refreshed; when the request expires or an error repeats,
                 the loop sends a stop instead of going silent.

Closed-loop ``forward_m`` / ``rotate_deg`` execution lives in the gRPC handler
thread and only feeds ``_ctrl_loop`` with a velocity, so a blocking move never
stalls the odometry stream or the lifecycle callbacks.
"""
from __future__ import annotations

import math
import threading
import time

from .tcp_client import ChassisError, LumiChassisClient

#: The chassis firmware clamps at 0.5 m/s and 1.0 rad/s; exceeding those is a
#: configuration error, not something to truncate silently.
FIRMWARE_MAX_LINEAR_M_S = 0.5
FIRMWARE_MAX_ANGULAR_RAD_S = 1.0

_STOP_RETRIES = 3
_WARN_INTERVAL_S = 5.0


def _wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ── config ───────────────────────────────────────────────────────────────
def _num(cfg: dict, key: str, default: float, lo=None, hi=None) -> float:
    raw = cfg.get(key, default)
    try:
        value = float(raw)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"`{key}` must be a number, got {raw!r}") from exc
    if not math.isfinite(value):
        raise ValueError(f"`{key}` must be finite, got {raw!r}")
    if lo is not None and value <= lo:
        raise ValueError(f"`{key}` must be > {lo}, got {value}")
    if hi is not None and value >= hi:
        raise ValueError(f"`{key}` must be < {hi}, got {value}")
    return value


def _flag(cfg: dict, key: str, default: bool) -> bool:
    raw = cfg.get(key, default)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        lowered = raw.strip().lower()
        if lowered in ("1", "true", "yes", "on"):
            return True
        if lowered in ("0", "false", "no", "off", ""):
            return False
    raise ValueError(f"`{key}` must be a boolean, got {raw!r}")


def _text(cfg: dict, key: str, default: str) -> str:
    raw = cfg.get(key, default)
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError(f"`{key}` must be a non-empty string, got {raw!r}")
    return raw.strip()


def parse_config(cfg: dict) -> dict:
    """Validate the deployment entry's ``config`` mapping.

    Returns a normalized dict. Raises ``ValueError`` with the offending key so
    on_init can surface a located error.
    """
    cfg = dict(cfg or {})

    max_linear = _num(cfg, "max_linear_m_s", 0.3, lo=0.0)
    if max_linear > FIRMWARE_MAX_LINEAR_M_S:
        raise ValueError(
            f"`max_linear_m_s` {max_linear} exceeds the chassis firmware limit "
            f"{FIRMWARE_MAX_LINEAR_M_S}"
        )
    max_angular = _num(cfg, "max_angular_rad_s", 0.5, lo=0.0)
    if max_angular > FIRMWARE_MAX_ANGULAR_RAD_S:
        raise ValueError(
            f"`max_angular_rad_s` {max_angular} exceeds the chassis firmware "
            f"limit {FIRMWARE_MAX_ANGULAR_RAD_S}"
        )

    joy_rate = _num(cfg, "joy_rate_hz", 5.0, lo=0.0)
    if joy_rate < 2.0:
        raise ValueError(
            f"`joy_rate_hz` must be >= 2.0 (one vendor joy command is valid "
            f"for 0.5 s), got {joy_rate}"
        )

    offset = cfg.get("vendor_pose_offset") or {}
    if not isinstance(offset, dict):
        raise ValueError("`vendor_pose_offset` must be a mapping {x, y, yaw}")
    pose_offset = (
        float(offset.get("x", 0.0)),
        float(offset.get("y", 0.0)),
        float(offset.get("yaw", 0.0)),
    )

    base_frame = _text(cfg, "base_frame", "base_link")
    odom_frame = _text(cfg, "odom_frame", "odom")
    if odom_frame == base_frame:
        raise ValueError(
            f"`odom_frame` and `base_frame` must differ (both are {base_frame!r})"
        )

    return {
        "host": _text(cfg, "host", "192.168.10.10"),
        "port": int(_num(cfg, "port", 31001, lo=0.0, hi=65536.0)),
        "cmd_timeout_s": _num(cfg, "cmd_timeout_s", 3.0, lo=0.0),
        "motion_enabled": _flag(cfg, "motion_enabled", False),
        "odom_frame": odom_frame,
        "base_frame": base_frame,
        "publish_tf": _flag(cfg, "publish_tf", True),
        "odom_rate_hz": _num(cfg, "odom_rate_hz", 10.0, lo=0.0),
        "max_linear_m_s": max_linear,
        "max_angular_rad_s": max_angular,
        "joy_rate_hz": joy_rate,
        "uuid": _text(cfg, "uuid", "lumi"),
        "vendor_pose_offset": pose_offset,
        "default_move_duration_s": _num(cfg, "default_move_duration_s", 1.0, lo=0.0),
        "move_timeout_s": _num(cfg, "move_timeout_s", 20.0, lo=0.0),
        "twist_timeout_s": _num(cfg, "twist_timeout_s", 0.5, lo=0.0),
        "move_tolerance_m": _num(cfg, "move_tolerance_m", 0.02, lo=0.0),
        "rotate_tolerance_deg": _num(cfg, "rotate_tolerance_deg", 2.0, lo=0.0),
        "stop_on_shutdown": _flag(cfg, "stop_on_shutdown", True),
    }


# ── driver ───────────────────────────────────────────────────────────────
class LumiChassisDriver:
    """One chassis. Owns the odom publisher, the velocity refresher and stop."""

    def __init__(self, cfg: dict, publish_odom, tf_pub=None, clock=None, log=print):
        self.cfg = cfg
        self._publish_odom = publish_odom
        self._tf_pub = tf_pub
        self._clock = clock
        self._log = log
        self._client = LumiChassisClient(
            host=cfg["host"],
            port=cfg["port"],
            timeout_s=cfg["cmd_timeout_s"],
            uuid=cfg["uuid"],
        )

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

        # pose in the published (Robonix) frame, and its freshness
        self._pose: tuple[float, float, float] | None = None
        self._pose_at = 0.0
        self._prev_pose: tuple[float, float, float] | None = None
        self._prev_at = 0.0
        self._status: dict = {}

        # commanded velocity: (linear, angular, expires_at)
        self._cmd: tuple[float, float, float] | None = None
        self._sent_motion = False
        self._consecutive_errors = 0
        self._last_warn = 0.0
        self._last_reject = 0.0

    # -- helpers ---------------------------------------------------------
    def _warn(self, msg: str) -> None:
        now = time.monotonic()
        if now - self._last_warn >= _WARN_INTERVAL_S:
            self._last_warn = now
            self._log(f"WARNING {msg}")

    def _stamp(self):
        if self._clock is None:
            return None
        return self._clock().now().to_msg()

    def _to_published_pose(self, vendor_pose: dict) -> tuple[float, float, float]:
        ox, oy, oyaw = self.cfg["vendor_pose_offset"]
        vx = float(vendor_pose.get("x", 0.0))
        vy = float(vendor_pose.get("y", 0.0))
        vth = float(vendor_pose.get("theta", 0.0))
        cos_o, sin_o = math.cos(oyaw), math.sin(oyaw)
        return (
            ox + vx * cos_o - vy * sin_o,
            oy + vx * sin_o + vy * cos_o,
            _wrap(vth + oyaw),
        )

    @staticmethod
    def _estop_engaged(status: dict) -> bool:
        for key in ("estop_state", "hard_estop_state", "soft_estop_state"):
            value = status.get(key)
            if isinstance(value, bool):
                if value:
                    return True
            elif isinstance(value, (int, float)) and value:
                return True
            elif isinstance(value, str) and value.strip().lower() in (
                "true", "1", "on", "yes", "engaged",
            ):
                return True
        return False

    def _block_reason(self) -> str | None:
        if not self.cfg["motion_enabled"]:
            return "motion_enabled is false (read-only mode)"
        if self._estop_engaged(self._status):
            return "chassis reports an engaged e-stop"
        return None

    def _sleep(self, seconds: float) -> bool:
        """Sleep that wakes early on stop. Returns True if stopping."""
        return self._stop.wait(seconds)

    # -- lifecycle -------------------------------------------------------
    def connect(self) -> dict:
        """Liveness check for CMD_INIT. Raises ChassisError when unreachable."""
        status = self._client.robot_status()
        with self._lock:
            self._status = status
            pose = status.get("current_pose")
            if isinstance(pose, dict):
                self._pose = self._to_published_pose(pose)
                self._pose_at = time.monotonic()
        return status

    def start(self) -> None:
        if self._threads:
            return  # idempotent ACTIVATE
        self._stop.clear()
        self._threads = [
            threading.Thread(target=self._odom_loop, name="lumi-odom", daemon=True),
            threading.Thread(target=self._ctrl_loop, name="lumi-ctrl", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        """Stop the machine first, then the threads."""
        self.stop_motion()
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads = []

    def shutdown(self) -> None:
        self.stop()
        if self.cfg["stop_on_shutdown"]:
            self._stop_hardware()

    def _stop_hardware(self) -> None:
        for _ in range(_STOP_RETRIES):
            try:
                self._client.stop()
            except Exception as exc:  # noqa: BLE001 - stop is best effort
                self._log(f"stop command failed: {exc}")
            time.sleep(0.05)

    # -- odometry --------------------------------------------------------
    def _odom_loop(self) -> None:
        period = 1.0 / self.cfg["odom_rate_hz"]
        while not self._stop.is_set():
            try:
                status = self._client.robot_status()
                self._ingest_status(status)
                self._consecutive_errors = 0
            except Exception as exc:  # noqa: BLE001 - keep the stream alive
                self._consecutive_errors += 1
                self._warn(
                    f"odom poll failed ({self._consecutive_errors}x): {exc}"
                )
                if self._consecutive_errors >= 3:
                    # Lose the pose rather than publish a stale one as current.
                    with self._lock:
                        self._pose = None
            self._sleep(period)

    def _ingest_status(self, status: dict) -> None:
        pose_raw = status.get("current_pose")
        if not isinstance(pose_raw, dict):
            return
        now = time.monotonic()
        pose = self._to_published_pose(pose_raw)

        with self._lock:
            self._status = status
            previous, previous_at = self._pose, self._pose_at
            self._pose, self._pose_at = pose, now

        twist = (0.0, 0.0)
        if previous is not None:
            dt = now - previous_at
            if dt > 1e-3:
                dx = pose[0] - previous[0]
                dy = pose[1] - previous[1]
                heading = previous[2]
                twist = (
                    (dx * math.cos(heading) + dy * math.sin(heading)) / dt,
                    _wrap(pose[2] - previous[2]) / dt,
                )

        self._publish(pose, twist)

    def _publish(self, pose: tuple[float, float, float], twist) -> None:
        stamp = self._stamp()
        try:
            self._publish_odom(
                self._make_odom(pose, twist, stamp)
            )
        except Exception as exc:  # noqa: BLE001
            self._warn(f"odom publish failed: {exc}")

        if self._tf_pub is None:
            return
        try:
            from geometry_msgs.msg import TransformStamped
            from tf2_msgs.msg import TFMessage

            tf = TransformStamped()
            if stamp is not None:
                tf.header.stamp = stamp
                tf.transform.header.stamp = stamp
            tf.header.frame_id = self.cfg["odom_frame"]
            tf.child_frame_id = self.cfg["base_frame"]
            tf.transform.translation.x = pose[0]
            tf.transform.translation.y = pose[1]
            tf.transform.rotation.z = math.sin(pose[2] / 2.0)
            tf.transform.rotation.w = math.cos(pose[2] / 2.0)
            self._tf_pub.publish(TFMessage(transforms=[tf]))
        except Exception as exc:  # noqa: BLE001
            self._warn(f"tf publish failed: {exc}")

    def _make_odom(self, pose, twist, stamp):
        from nav_msgs.msg import Odometry

        msg = Odometry()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.header.frame_id = self.cfg["odom_frame"]
        msg.child_frame_id = self.cfg["base_frame"]
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        msg.twist.twist.linear.x = twist[0]
        msg.twist.twist.angular.z = twist[1]

        # The vendor SLAM pose is a fused estimate, not wheel odometry: report
        # a modest covariance on planar x/y/yaw and "unknown" elsewhere, so
        # consumers do not trust z/roll/pitch it never measured.
        pose_cov = [0.0] * 36
        for index, variance in ((0, 0.02), (7, 0.02), (35, 0.05)):
            pose_cov[index] = variance
        for index in (14, 21, 28):
            pose_cov[index] = 1e6
        msg.pose.covariance = pose_cov
        return msg

    # -- velocity control ------------------------------------------------
    def command_velocity(self, linear: float, angular: float, hold_s: float) -> None:
        """Request a velocity for ``hold_s`` seconds (twist_in and move feed this)."""
        reason = self._block_reason()
        if reason:
            now = time.monotonic()
            if now - self._last_reject >= _WARN_INTERVAL_S:
                self._last_reject = now
                self._log(f"velocity command rejected: {reason}")
            return

        linear = max(-self.cfg["max_linear_m_s"], min(self.cfg["max_linear_m_s"], float(linear)))
        angular = max(-self.cfg["max_angular_rad_s"], min(self.cfg["max_angular_rad_s"], float(angular)))
        with self._lock:
            self._cmd = (linear, angular, time.monotonic() + max(0.05, float(hold_s)))

    def stop_motion(self) -> None:
        """Forget any requested velocity; the control loop sends the stop."""
        with self._lock:
            self._cmd = None

    def _ctrl_loop(self) -> None:
        period = 1.0 / self.cfg["joy_rate_hz"]
        while not self._stop.is_set():
            now = time.monotonic()
            with self._lock:
                cmd = self._cmd
                if cmd is not None and now >= cmd[2]:
                    self._cmd = None
                    cmd = None

            if cmd is None:
                if self._sent_motion:
                    try:
                        self._client.stop()
                        self._sent_motion = False
                    except Exception as exc:  # noqa: BLE001
                        self._warn(f"stop failed: {exc}")
                self._sleep(period)
                continue

            reason = self._block_reason()
            if reason:
                self.stop_motion()
                self._log(f"motion aborted: {reason}")
                self._sleep(period)
                continue

            try:
                self._client.joy(cmd[0], cmd[1])
                self._sent_motion = True
            except Exception as exc:  # noqa: BLE001
                self._warn(f"joy_control failed: {exc}")
                self.stop_motion()
            self._sleep(period)

    # -- closed-loop moves -----------------------------------------------
    def _fresh_pose(self) -> tuple[float, float, float] | None:
        with self._lock:
            pose, at = self._pose, self._pose_at
        if pose is not None and time.monotonic() - at <= 0.5:
            return pose
        try:
            status = self._client.robot_status()
        except ChassisError:
            return None
        raw = status.get("current_pose")
        if not isinstance(raw, dict):
            return None
        pose = self._to_published_pose(raw)
        with self._lock:
            self._pose, self._pose_at = pose, time.monotonic()
            self._status = status
        return pose

    def execute_move(
        self,
        forward_m: float = 0.0,
        rotate_deg: float = 0.0,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        angular_x: float = 0.0,
        angular_y: float = 0.0,
        angular_z: float = 0.0,
        duration_sec: float = 0.0,
    ) -> dict:
        """Handle one ExecuteMoveCommand. Returns a JSON-able status dict."""
        reason = self._block_reason()
        if reason:
            return {"status": "rejected", "detail": reason}

        if self._status.get("move_status") == "running":
            return {
                "status": "rejected",
                "detail": "a vendor navigation task is already running; "
                          "cancel it (/api/move/cancel) before sending motion",
            }

        if abs(forward_m) > 1e-9:
            return self._closed_loop_forward(forward_m)
        if abs(rotate_deg) > 1e-9:
            return self._closed_loop_rotate(rotate_deg)
        return self._velocity_burst(
            linear_x, linear_y, linear_z, angular_x, angular_y, angular_z, duration_sec
        )

    def _velocity_burst(self, lx, ly, lz, ax, ay, az, duration_sec) -> dict:
        if any(abs(v) > 1e-9 for v in (ly, lz, ax, ay)):
            return {
                "status": "rejected",
                "detail": "only planar linear.x and angular.z are supported "
                          "by a differential-drive base",
            }
        if abs(lx) < 1e-9 and abs(az) < 1e-9:
            return {"status": "rejected", "detail": "no motion requested"}

        hold = float(duration_sec) if duration_sec > 0 else self.cfg["default_move_duration_s"]
        hold = min(hold, self.cfg["move_timeout_s"])
        linear = max(-self.cfg["max_linear_m_s"], min(self.cfg["max_linear_m_s"], lx))
        angular = max(-self.cfg["max_angular_rad_s"], min(self.cfg["max_angular_rad_s"], az))

        self.command_velocity(linear, angular, hold)
        try:
            if self._sleep(hold):
                return {"status": "aborted", "detail": "provider shutting down"}
        finally:
            self.stop_motion()
        return {
            "status": "accepted",
            "detail": f"drove linear={linear:.3f} m/s angular={angular:.3f} rad/s "
                      f"for {hold:.2f} s",
        }

    def _closed_loop_forward(self, meters: float) -> dict:
        start = self._fresh_pose()
        if start is None:
            return {"status": "rejected", "detail": "no chassis pose available"}
        x0, y0, heading = start
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        tolerance = self.cfg["move_tolerance_m"]
        speed = min(self.cfg["max_linear_m_s"], max(0.03, abs(meters) / 2.0))
        expected = abs(meters) / speed
        deadline = time.monotonic() + min(self.cfg["move_timeout_s"], expected * 3.0 + 5.0)

        direction = 1.0 if meters > 0 else -1.0
        self.command_velocity(direction * speed, 0.0, 0.5)
        try:
            while time.monotonic() < deadline:
                if self._sleep(0.1):
                    return {"status": "aborted", "detail": "provider shutting down"}
                pose = self._fresh_pose()
                if pose is None:
                    return {"status": "failed", "detail": "lost chassis pose"}
                travelled = (pose[0] - x0) * cos_h + (pose[1] - y0) * sin_h
                if abs(travelled) >= abs(meters) - tolerance:
                    return {
                        "status": "accepted",
                        "detail": f"travelled {travelled:+.3f} m "
                                  f"(target {meters:+.3f} m)",
                    }
        finally:
            self.stop_motion()
        return {"status": "timeout", "detail": f"did not reach {meters:+.3f} m in time"}

    def _closed_loop_rotate(self, degrees: float) -> dict:
        start = self._fresh_pose()
        if start is None:
            return {"status": "rejected", "detail": "no chassis pose available"}
        target = math.radians(degrees)
        tolerance = math.radians(self.cfg["rotate_tolerance_deg"])
        speed = min(self.cfg["max_angular_rad_s"], max(math.radians(10.0),
                                                       abs(target) / 2.0))
        expected = abs(target) / speed
        deadline = time.monotonic() + min(self.cfg["move_timeout_s"], expected * 3.0 + 5.0)

        direction = 1.0 if target > 0 else -1.0
        self.command_velocity(0.0, direction * speed, 0.5)
        turned = 0.0
        previous = start[2]
        try:
            while time.monotonic() < deadline:
                if self._sleep(0.1):
                    return {"status": "aborted", "detail": "provider shutting down"}
                pose = self._fresh_pose()
                if pose is None:
                    return {"status": "failed", "detail": "lost chassis pose"}
                turned += _wrap(pose[2] - previous)
                previous = pose[2]
                if direction * turned >= abs(target) - tolerance:
                    return {
                        "status": "accepted",
                        "detail": f"rotated {math.degrees(turned):+.1f} deg "
                                  f"(target {degrees:+.1f} deg)",
                    }
        finally:
            self.stop_motion()
        return {"status": "timeout", "detail": f"did not rotate {degrees:+.1f} deg in time"}
