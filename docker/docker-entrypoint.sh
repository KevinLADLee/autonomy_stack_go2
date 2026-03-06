#!/bin/bash

# Docker entrypoint script for autonomy_stack_go2
# This script sets up the ROS2 environment and can launch the system

set -e

# Fix permissions for mounted volumes if running as root
# Only fix permissions for src and bags (mounted from host)
if [ "$(id -u)" = "0" ] && [ -n "${HOST_UID}" ] && [ -n "${HOST_GID}" ]; then
    # Fix permissions for mounted directories only
    for dir in /workspace/src /workspace/bags; do
        if [ -d "$dir" ]; then
            # Only fix if directory is owned by root
            if [ "$(stat -c '%u' "$dir" 2>/dev/null)" = "0" ]; then
                chown -R ${HOST_UID}:${HOST_GID} "$dir" 2>/dev/null || true
            fi
        fi
    done
    # Create bags directory if it doesn't exist
    mkdir -p /workspace/bags
    chown -R ${HOST_UID}:${HOST_GID} /workspace/bags 2>/dev/null || true
fi

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

# Set ROS domain ID if provided
if [ -n "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
fi

# Set RMW implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# If command is provided, execute it
if [ $# -gt 0 ]; then
    exec "$@"
else
    # Default: start bash shell
    exec /bin/bash
fi

