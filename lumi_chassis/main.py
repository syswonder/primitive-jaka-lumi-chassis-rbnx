# SPDX-License-Identifier: Apache-2.0
"""lumi_chassis — Robonix chassis primitive for the JAKA Lumi mobile base.

Owns ``robonix/primitive/chassis/*``:

  odom      topic_out  ROS 2 /odom      nav_msgs/Odometry (vendor SLAM pose)
  twist_in  topic_in   ROS 2 /cmd_vel   geometry_msgs/Twist
  move      rpc        gRPC ExecuteMoveCommand -> std_msgs/String status JSON

Motion is rejected while ``motion_enabled`` is false (the default) — a real
safety gate, not a fake disable. See config.spec and CAPABILITY.md.
"""
from __future__ import annotations

import json
import logging

from robonix_api import Err, Ok, Primitive

provider = Primitive(id="lumi_chassis", namespace="robonix/primitive/chassis")

# Imported AFTER Primitive(): its constructor puts the package's codegen
# output on sys.path, which is where chassis_pb2 / std_msgs_pb2 live.
import chassis_pb2  # type: ignore  # noqa: E402
import std_msgs_pb2  # type: ignore  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

from .driver import LumiChassisDriver, parse_config  # noqa: E402

log = logging.getLogger("lumi_chassis")
logging.basicConfig(
    level=logging.INFO, format="[lumi_chassis] %(levelname)s %(message)s"
)

_ODOM_CONTRACT = "robonix/primitive/chassis/odom"
_TWIST_CONTRACT = "robonix/primitive/chassis/twist_in"
_MOVE_CONTRACT = "robonix/primitive/chassis/move"

_driver: LumiChassisDriver | None = None


def _emit_odom(msg) -> None:
    provider.emit(_ODOM_CONTRACT, msg)


def _status_response(payload: dict):
    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps(payload, ensure_ascii=False))
    )


# ── gRPC: chassis/move ───────────────────────────────────────────────────
@provider.grpc(_MOVE_CONTRACT)
def move(req, ctx):
    """One-shot bounded motion: forward_m | rotate_deg | velocity burst."""
    if _driver is None:
        return _status_response(
            {"status": "rejected", "detail": "provider is not initialised"}
        )

    command = req.command
    try:
        result = _driver.execute_move(
            forward_m=command.forward_m,
            rotate_deg=command.rotate_deg,
            linear_x=command.linear_x,
            linear_y=command.linear_y,
            linear_z=command.linear_z,
            angular_x=command.angular_x,
            angular_y=command.angular_y,
            angular_z=command.angular_z,
            duration_sec=command.duration_sec,
        )
    except Exception as exc:  # noqa: BLE001 - never drop the RPC
        log.warning("move failed: %s", exc)
        result = {"status": "failed", "detail": f"{type(exc).__name__}: {exc}"}

    log.info("move -> %s", result)
    return _status_response(result)


# ── twist_in callback ────────────────────────────────────────────────────
def _on_twist(msg: Twist) -> None:
    if _driver is None:
        return
    _driver.command_velocity(
        msg.linear.x, msg.angular.z, _driver.cfg["twist_timeout_s"]
    )


# ── Lifecycle ────────────────────────────────────────────────────────────
@provider.on_init
def init(cfg: dict):
    global _driver

    try:
        normalized = parse_config(cfg)
    except ValueError as exc:
        return Err(f"invalid config: {exc}")

    provider.create_publisher(
        contract_id=_ODOM_CONTRACT,
        topic="/odom",
        msg_type=Odometry,
        qos="reliable",
        description=(
            "Chassis pose from the vendor SLAM in the odom frame "
            f"(child: {normalized['base_frame']}). No wheel encoders are "
            "exposed, so the residual drift is whatever the vendor map has."
        ),
    )
    provider.create_subscription(
        contract_id=_TWIST_CONTRACT,
        topic="/cmd_vel",
        msg_type=Twist,
        callback=_on_twist,
        qos="reliable",
    )

    tf_pub = None
    clock = None
    try:
        from robonix_api.ros import RosBackend

        backend = RosBackend.get()
        node = backend.node
        clock = node.get_clock
        if normalized["publish_tf"]:
            from tf2_msgs.msg import TFMessage

            # Not a capability contract: /tf is a ROS-wide convention and the
            # chassis owns exactly this one transform.
            tf_pub = backend.create_publisher(TFMessage, "/tf", 100)
    except Exception as exc:  # noqa: BLE001
        return Err(f"failed to start the ROS 2 backend: {exc}")

    driver = LumiChassisDriver(
        normalized,
        publish_odom=_emit_odom,
        tf_pub=tf_pub,
        clock=clock,
        log=lambda text: log.info("%s", text),
    )
    try:
        status = driver.connect()
    except Exception as exc:  # noqa: BLE001
        return Err(
            f"chassis unreachable at {normalized['host']}:{normalized['port']} "
            f"(check the 192.168.10.0/24 static address on the WiFi link): {exc}"
        )

    _driver = driver
    log.info(
        "CMD_INIT ok: endpoint=%s:%s motion_enabled=%s tf=%s pose=%s",
        normalized["host"],
        normalized["port"],
        normalized["motion_enabled"],
        tf_pub is not None,
        status.get("current_pose"),
    )
    return Ok()


@provider.on_activate
def activate():
    if _driver is None:
        return Err("provider is not initialised")
    _driver.start()
    log.info(
        "CMD_ACTIVATE ok: odom %.1f Hz, joy %.1f Hz, estop=%s move_status=%s",
        _driver.cfg["odom_rate_hz"],
        _driver.cfg["joy_rate_hz"],
        _driver._status.get("estop_state"),
        _driver._status.get("move_status"),
    )
    return Ok()


@provider.on_deactivate
def deactivate():
    if _driver is not None:
        _driver.stop()
        log.info("CMD_DEACTIVATE ok")
    return Ok()


@provider.on_shutdown
def shutdown():
    if _driver is not None:
        _driver.shutdown()
        log.info("CMD_SHUTDOWN ok")
    return Ok()


if __name__ == "__main__":
    provider.run()
