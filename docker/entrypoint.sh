#!/usr/bin/env bash

set -e

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # ROS remains an optional image variant. Source its environment only when
    # the selected base image actually provides a ROS distribution.
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

exec "$@"
