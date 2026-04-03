#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKSPACE_SETUP="$WORKSPACE_ROOT/install/setup.bash"

safe_source() {
	local target="$1"
	set +u
	# shellcheck disable=SC1090
	source "$target"
	set -u
}

for ros_setup in /opt/ros/humble/setup.bash /opt/ros/foxy/setup.bash; do
	if [[ -f "$ros_setup" ]]; then
		safe_source "$ros_setup"
		break
	fi
done

if ! command -v ros2 >/dev/null 2>&1; then
	echo "ros2 command not found after sourcing the ROS environment" >&2
	exit 1
fi

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
	echo "Workspace setup file not found: $WORKSPACE_SETUP" >&2
	exit 1
fi

safe_source "$WORKSPACE_SETUP"

ros2 topic pub /utlidar/switch std_msgs/msg/String "data: 'OFF'" \
	--times 10 \
	--wait-matching-subscriptions 1 \
	--keep-alive 1 \
	--node-name switch_off_utlidar_boot