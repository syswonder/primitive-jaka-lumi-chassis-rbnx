#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Start the chassis provider in the foreground. rbnx start / rbnx boot drive
# the lifecycle; this script only prepares the ROS environment.
set -euo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"

# ROS setup.bash references unset variables under `set -u`; relax it around
# the source calls.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"
set -u

cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

exec "$PKG_ROOT/.venv/bin/python" -m lumi_chassis.main
