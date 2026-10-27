# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the vendor JSON-over-TCP client against a fake socket server.

No chassis required: a threaded listener on 127.0.0.1 replays canned replies
so the framing rules (partial reads, FAIL payloads, silent closes) are pinned
down.
"""
import json
import socket
import threading
import time
import unittest

from lumi_chassis.tcp_client import ChassisError, LumiChassisClient


class FakeChassisServer(threading.Thread):
    """Accept one command per connection and answer with a canned payload."""

    def __init__(self, reply, split_at=None, close_without_reply=False):
        super().__init__(daemon=True)
        self._reply = reply
        self._split_at = split_at
        self._close_without_reply = close_without_reply
        self.received = []
        self._stop = threading.Event()
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("127.0.0.1", 0))
        self._srv.listen(8)
        self._srv.settimeout(0.2)
        self.port = self._srv.getsockname()[1]

    def run(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with conn:
                data = b""
                while b"?" not in data and not data:
                    # one sendall from the client; a single recv is enough
                    data = conn.recv(65536)
                    if not data:
                        break
                self.received.append(data.decode("utf-8"))
                if self._close_without_reply:
                    continue
                payload = (
                    self._reply
                    if isinstance(self._reply, bytes)
                    else self._reply.encode("utf-8")
                )
                if self._split_at:
                    conn.sendall(payload[: self._split_at])
                    time.sleep(0.05)
                    conn.sendall(payload[self._split_at :])
                else:
                    conn.sendall(payload)

    def close(self):
        self._stop.set()
        self._srv.close()


def _client(server, timeout_s=2.0, uuid="lumi"):
    return LumiChassisClient(
        "127.0.0.1", server.port, timeout_s=timeout_s, uuid=uuid
    )


class TestRoundTrip(unittest.TestCase):
    def test_ok_reply_is_unwrapped(self):
        reply = json.dumps(
            {
                "status": "OK",
                "command": "/api/robot_status",
                "results": {"current_pose": {"x": 1.5, "y": -2.0, "theta": 0.5}},
            }
        )
        server = FakeChassisServer(reply)
        server.start()
        try:
            status = _client(server).robot_status()
        finally:
            server.close()
        self.assertEqual(status["current_pose"]["x"], 1.5)
        self.assertEqual(server.received, ["/api/robot_status"])

    def test_fragmented_reply_is_reassembled(self):
        reply = json.dumps(
            {"status": "OK", "results": {"power_percent": 80.0}}
        )
        server = FakeChassisServer(reply, split_at=len(reply) // 2)
        server.start()
        try:
            status = _client(server).robot_status()
        finally:
            server.close()
        self.assertEqual(status["power_percent"], 80.0)

    def test_fail_status_raises(self):
        reply = json.dumps(
            {"status": "FAIL", "error_message": "moving, keep out"}
        )
        server = FakeChassisServer(reply)
        server.start()
        try:
            with self.assertRaises(ChassisError) as ctx:
                _client(server).joy(0.1, 0.0)
        finally:
            server.close()
        self.assertIn("moving, keep out", str(ctx.exception))

    def test_silent_close_raises(self):
        server = FakeChassisServer("", close_without_reply=True)
        server.start()
        try:
            with self.assertRaises(ChassisError) as ctx:
                _client(server).robot_status()
        finally:
            server.close()
        self.assertIn("no reply", str(ctx.exception))

    def test_unparsable_reply_raises(self):
        server = FakeChassisServer("<html>nope</html>")
        server.start()
        try:
            with self.assertRaises(ChassisError) as ctx:
                _client(server).robot_status()
        finally:
            server.close()
        self.assertIn("not valid JSON", str(ctx.exception))


class TestCommandEncoding(unittest.TestCase):
    def _run(self, call, reply='{"status": "OK", "results": {}}'):
        server = FakeChassisServer(reply)
        server.start()
        try:
            client = _client(server)
            call(client)
        finally:
            server.close()
        return server.received[0]

    def test_joy_encodes_signed_floats_and_uuid(self):
        cmd = self._run(lambda c: c.joy(-0.125, 0.5))
        self.assertEqual(
            cmd,
            "/api/joy_control?angular_velocity=0.5000"
            "&linear_velocity=-0.1250&uuid=lumi",
        )

    def test_stop_is_zero_velocity(self):
        cmd = self._run(lambda c: c.stop())
        self.assertIn("angular_velocity=0.0000", cmd)
        self.assertIn("linear_velocity=0.0000", cmd)

    def test_move_to_location_keeps_raw_numbers(self):
        cmd = self._run(lambda c: c.move_to_location(1.5, -2.0, 0.25))
        self.assertEqual(cmd, "/api/move?location=1.5,-2.0,0.25&uuid=lumi")

    def test_estop_flag_is_lowercase(self):
        self.assertIn("flag=true", self._run(lambda c: c.estop(True)))
        self.assertIn("flag=false", self._run(lambda c: c.estop(False)))

    def test_cancel_move(self):
        self.assertEqual(
            self._run(lambda c: c.cancel_move()),
            "/api/move/cancel?uuid=lumi",
        )


if __name__ == "__main__":
    unittest.main()
