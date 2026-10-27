# SPDX-License-Identifier: Apache-2.0
"""JSON-over-TCP client for the JAKA Lumi chassis controller.

Protocol (vendor "AGV API" manual, base board 192.168.10.10:31001):

  * one command per TCP connection;
  * a command is a URL-like ASCII string, e.g. ``/api/robot_status`` or
    ``/api/joy_control?linear_velocity=0.1&angular_velocity=0``;
  * the reply is a single JSON object::

        {"status": "OK", "command": "/api/robot_status", "results": {...}}

    or, on failure::

        {"status": "FAIL", "command": ..., "error_message": "..."}

The base closes the connection after the reply, so a fresh socket per
command is the intended usage (the vendor's own demo does the same).
"""
from __future__ import annotations

import json
import socket
import time


class ChassisError(RuntimeError):
    """The chassis rejected a command, or answered nothing parsable."""


class LumiChassisClient:
    """Blocking, thread-safe-by-construction client (no shared socket state)."""

    def __init__(
        self,
        host: str,
        port: int = 31001,
        timeout_s: float = 3.0,
        uuid: str = "lumi",
    ) -> None:
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.uuid = uuid

    @property
    def endpoint(self) -> str:
        return f"{self.host}:{self.port}"

    # ── low level ────────────────────────────────────────────────────────
    def _roundtrip(self, cmd: str) -> dict:
        """Send one command and return the decoded JSON reply."""
        deadline = time.monotonic() + self.timeout_s
        buf = b""
        try:
            with socket.create_connection(
                (self.host, self.port), timeout=self.timeout_s
            ) as sock:
                sock.sendall(cmd.encode("utf-8"))
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    sock.settimeout(max(0.05, remaining))
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        break
                    if not chunk:
                        break
                    buf += chunk
                    try:
                        return json.loads(buf.decode("utf-8"))
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        # Partial frame: keep reading until the object closes.
                        continue
        except OSError as exc:
            raise ChassisError(
                f"{cmd}: transport error talking to {self.endpoint}: {exc}"
            ) from exc

        if not buf:
            raise ChassisError(
                f"{cmd}: no reply from {self.endpoint} (connection closed)"
            )
        raise ChassisError(f"{cmd}: reply is not valid JSON: {buf[:200]!r}")

    def _call(self, cmd: str) -> dict:
        """Send one command and unwrap ``results``, raising on any failure."""
        reply = self._roundtrip(cmd)
        status = reply.get("status")
        if status != "OK":
            raise ChassisError(
                f"{cmd}: chassis returned {status!r}: "
                f"{reply.get('error_message')!r}"
            )
        results = reply.get("results")
        return results if isinstance(results, dict) else {}

    # ── read ─────────────────────────────────────────────────────────────
    def robot_status(self) -> dict:
        """Pose, battery, estop and motion state. Also the liveness probe."""
        return self._call("/api/robot_status")

    def robot_info(self) -> dict:
        return self._call("/api/robot_info")

    def markers(self) -> dict:
        return self._call("/api/markers/query_brief")

    def distance_probe(self, x: float, y: float) -> dict:
        """Obstacle distance at a map-frame point (static map + sensors)."""
        return self._call(f"/api/map/distance_probe?x={x:.3f}&y={y:.3f}")

    # ── motion ───────────────────────────────────────────────────────────
    def joy(self, linear_m_s: float, angular_rad_s: float) -> None:
        """Direct velocity command. Each call stays valid for only 0.5 s."""
        self._call(
            "/api/joy_control"
            f"?angular_velocity={angular_rad_s:.4f}"
            f"&linear_velocity={linear_m_s:.4f}"
            f"&uuid={self.uuid}"
        )

    def stop(self) -> None:
        """Zero velocity. Callers retry it; never raises on a healthy base."""
        self.joy(0.0, 0.0)

    def estop(self, engaged: bool) -> dict:
        flag = "true" if engaged else "false"
        return self._call(f"/api/estop?flag={flag}&uuid={self.uuid}")

    def move_to_location(self, x: float, y: float, theta: float) -> dict:
        """Vendor planning + obstacle avoidance to a vendor-map pose."""
        return self._call(f"/api/move?location={x},{y},{theta}&uuid={self.uuid}")

    def move_to_marker(self, marker: str) -> dict:
        return self._call(f"/api/move?marker={marker}&uuid={self.uuid}")

    def cancel_move(self) -> dict:
        return self._call(f"/api/move/cancel?uuid={self.uuid}")
