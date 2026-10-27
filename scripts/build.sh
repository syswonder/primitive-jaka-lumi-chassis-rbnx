#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Build: package venv + codegen (proto/grpc stubs, MCP types, ROS 2 IDL
# overlay) + colcon build of that overlay.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"
VENV="$PKG/.venv"

# System-site-packages so rclpy / nav_msgs / tf2_msgs resolve from the host
# ROS 2 install instead of being reinstalled into the venv.
if [[ ! -x "$VENV/bin/python" ]]; then
    python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/pip" install -q --disable-pip-version-check -r "$PKG/requirements.txt"

# ROS setup.bash references unset variables under `set -u`; relax it around
# the source calls.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

rbnx codegen -p "$PKG" --mcp --ros2 --python "$VENV/bin/python"

(
    cd "$PKG/rbnx-build/codegen/ros2_idl"
    colcon build
)

echo "[lumi_chassis] build done"
