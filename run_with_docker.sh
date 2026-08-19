#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./run_with_docker.sh PROFILE [SCRIPT.py [ARGS...]]
  ./run_with_docker.sh PROFILE COMMAND [ARGS...]

Profiles: core, default, mujoco, isaac, ros, wsl

With no command, the selected container opens an interactive shell. A first
argument ending in .py is run with Python for compatibility with SPARK v1.
EOF
}

profile="${1:-default}"
if [[ $# -gt 0 ]]; then
    shift
fi

case "${profile}" in
    core)
        service="spark-core"
        compose_profile="core"
        ;;
    default|mujoco)
        service="spark-default"
        compose_profile="default"
        ;;
    isaac|ros|wsl)
        service="spark-${profile}"
        compose_profile="${profile}"
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        usage >&2
        echo "Error: unknown Docker profile: ${profile}" >&2
        exit 2
        ;;
esac

if [[ $# -eq 0 ]]; then
    command=(bash)
elif [[ "$1" == *.py ]]; then
    command=(python "$@")
else
    command=("$@")
fi

exec docker compose --profile "${compose_profile}" run \
    --rm --remove-orphans "${service}" "${command[@]}"
