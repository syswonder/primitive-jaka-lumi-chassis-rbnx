# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the chassis driver against a fake vendor client.

Covers config validation, the vendor-pose -> odom-pose transform, the safety
gates and the closed-loop move accounting. No ROS, no hardware.
"""
import math
import unittest

from lumi_chassis import driver as driver_mod
from lumi_chassis.driver import LumiChassisDriver, parse_config


class FakeClient:
    """Stand-in for LumiChassisClient: scripted status, recorded commands."""

    def __init__(self, poses=None, status_extra=None):
        self.poses = list(poses or [(0.0, 0.0, 0.0)])
        self.status_extra = dict(status_extra or {})
        self.joy_calls = []
        self.stops = 0
        self.status_calls = 0

    def robot_status(self):
        self.status_calls += 1
        if len(self.poses) > 1:
            x, y, theta = self.poses.pop(0)
        else:
            x, y, theta = self.poses[0]
        status = {"current_pose": {"x": x, "y": y, "theta": theta}}
        status.update(self.status_extra)
        return status

    def joy(self, linear, angular):
        self.joy_calls.append((linear, angular))

    def stop(self):
        self.stops += 1

    def estop(self, engaged):
        return {"flag": engaged}


def _driver(cfg=None, client=None):
    normalized = parse_config(cfg or {"motion_enabled": True})
    published = []
    drv = LumiChassisDriver(
        normalized,
        publish_odom=published.append,
        tf_pub=None,
        clock=None,
        log=lambda text: None,
    )
    drv._client = client or FakeClient()
    return drv, published


class TestParseConfig(unittest.TestCase):
    def test_defaults_are_read_only(self):
        cfg = parse_config({})
        self.assertFalse(cfg["motion_enabled"])
        self.assertEqual(cfg["host"], "192.168.10.10")
        self.assertEqual(cfg["port"], 31001)
        self.assertEqual(cfg["odom_frame"], "odom")
        self.assertEqual(cfg["base_frame"], "base_link")
        self.assertTrue(cfg["publish_tf"])
        self.assertEqual(cfg["vendor_pose_offset"], (0.0, 0.0, 0.0))

    def test_linear_speed_above_firmware_limit_is_rejected(self):
        with self.assertRaises(ValueError) as ctx:
            parse_config({"max_linear_m_s": 0.6})
        self.assertIn("firmware limit", str(ctx.exception))

    def test_angular_speed_above_firmware_limit_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_config({"max_angular_rad_s": 1.5})

    def test_joy_rate_below_two_hertz_is_rejected(self):
        with self.assertRaises(ValueError) as ctx:
            parse_config({"joy_rate_hz": 1.0})
        self.assertIn(">= 2.0", str(ctx.exception))

    def test_identical_frames_are_rejected(self):
        with self.assertRaises(ValueError) as ctx:
            parse_config({"odom_frame": "base_link"})
        self.assertIn("must differ", str(ctx.exception))

    def test_non_numeric_value_names_the_key(self):
        with self.assertRaises(ValueError) as ctx:
            parse_config({"port": "abc"})
        self.assertIn("`port`", str(ctx.exception))

    def test_string_boolean_is_accepted(self):
        self.assertTrue(parse_config({"motion_enabled": "true"})["motion_enabled"])
        self.assertFalse(parse_config({"motion_enabled": "0"})["motion_enabled"])

    def test_bad_boolean_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_config({"publish_tf": "maybe"})

    def test_bad_offset_type_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_config({"vendor_pose_offset": [1, 2, 3]})


class TestPoseTransform(unittest.TestCase):
    def test_identity_offset_matches_vendor_pose(self):
        drv, _ = _driver()
        self.assertEqual(
            drv._to_published_pose({"x": 1.0, "y": 2.0, "theta": 0.3}),
            (1.0, 2.0, 0.3),
        )

    def test_offset_rotates_then_translates(self):
        drv, _ = _driver(
            {"vendor_pose_offset": {"x": 10.0, "y": 20.0, "yaw": math.pi / 2}}
        )
        x, y, theta = drv._to_published_pose({"x": 1.0, "y": 0.0, "theta": 0.0})
        self.assertAlmostEqual(x, 10.0, places=6)
        self.assertAlmostEqual(y, 21.0, places=6)
        self.assertAlmostEqual(theta, math.pi / 2, places=6)

    def test_theta_is_wrapped(self):
        drv, _ = _driver(
            {"vendor_pose_offset": {"x": 0.0, "y": 0.0, "yaw": math.pi}}
        )
        _, _, theta = drv._to_published_pose({"x": 0.0, "y": 0.0, "theta": math.pi})
        self.assertLessEqual(abs(theta), math.pi)


class TestEstopDetection(unittest.TestCase):
    def test_numeric_and_boolean_and_string_shapes(self):
        self.assertTrue(LumiChassisDriver._estop_engaged({"estop_state": 1}))
        self.assertTrue(LumiChassisDriver._estop_engaged({"estop_state": True}))
        self.assertTrue(
            LumiChassisDriver._estop_engaged({"hard_estop_state": "true"})
        )
        self.assertTrue(
            LumiChassisDriver._estop_engaged({"soft_estop_state": "engaged"})
        )
        self.assertFalse(LumiChassisDriver._estop_engaged({"estop_state": 0}))
        self.assertFalse(LumiChassisDriver._estop_engaged({}))


class TestMotionGates(unittest.TestCase):
    def test_read_only_rejects_everything(self):
        drv, _ = _driver({"motion_enabled": False})
        result = drv.execute_move(forward_m=0.1)
        self.assertEqual(result["status"], "rejected")
        self.assertIn("motion_enabled", result["detail"])

        drv.command_velocity(0.1, 0.0, 0.5)
        self.assertIsNone(drv._cmd)

    def test_estop_rejects(self):
        drv, _ = _driver({"motion_enabled": True})
        drv._status = {"estop_state": 1}
        result = drv.execute_move(forward_m=0.1)
        self.assertEqual(result["status"], "rejected")
        self.assertIn("e-stop", result["detail"])

    def test_running_vendor_task_rejects(self):
        drv, _ = _driver({"motion_enabled": True})
        drv._status = {"estop_state": 0, "move_status": "running"}
        result = drv.execute_move(forward_m=0.1)
        self.assertEqual(result["status"], "rejected")
        self.assertIn("vendor navigation", result["detail"])

    def test_command_velocity_is_clamped_to_config(self):
        drv, _ = _driver(
            {"motion_enabled": True, "max_linear_m_s": 0.2, "max_angular_rad_s": 0.4}
        )
        drv.command_velocity(5.0, -5.0, 0.5)
        linear, angular, _ = drv._cmd
        self.assertEqual(linear, 0.2)
        self.assertEqual(angular, -0.4)

    def test_non_planar_velocity_is_rejected(self):
        drv, _ = _driver({"motion_enabled": True})
        result = drv.execute_move(linear_y=0.1)
        self.assertEqual(result["status"], "rejected")
        self.assertIn("differential-drive", result["detail"])

    def test_empty_velocity_request_is_rejected(self):
        drv, _ = _driver({"motion_enabled": True})
        self.assertEqual(drv.execute_move()["status"], "rejected")


class TestClosedLoop(unittest.TestCase):
    def test_forward_counts_progress_along_heading(self):
        # 0.1 m target, 0.02 m per status call -> arrives on the 5th sample
        poses = [(0.02 * i, 0.0, 0.0) for i in range(6)]
        drv, _ = _driver({"motion_enabled": True}, client=FakeClient(poses))
        result = drv.execute_move(forward_m=0.1)
        self.assertEqual(result["status"], "accepted")
        self.assertIn("travelled", result["detail"])
        self.assertIsNone(drv._cmd)

    def test_forward_ignores_lateral_displacement(self):
        # Walking sideways must not satisfy a forward target.
        poses = [(0.0, 0.02 * i, 0.0) for i in range(6)]
        drv, _ = _driver(
            {"motion_enabled": True, "move_timeout_s": 0.4}, client=FakeClient(poses)
        )
        self.assertEqual(drv.execute_move(forward_m=0.1)["status"], "timeout")

    def test_rotate_arrives_on_angle(self):
        poses = [(0.0, 0.0, math.radians(3.0 * i)) for i in range(8)]
        drv, _ = _driver({"motion_enabled": True}, client=FakeClient(poses))
        result = drv.execute_move(rotate_deg=10.0)
        self.assertEqual(result["status"], "accepted")
        self.assertIn("rotated", result["detail"])

    def test_forward_without_pose_is_rejected(self):
        client = FakeClient([(0.0, 0.0, 0.0)])

        def _boom():
            raise driver_mod.ChassisError("unreachable")

        client.robot_status = _boom  # type: ignore[assignment]
        drv, _ = _driver({"motion_enabled": True}, client=client)
        self.assertEqual(drv.execute_move(forward_m=0.1)["status"], "rejected")


if __name__ == "__main__":
    unittest.main()
