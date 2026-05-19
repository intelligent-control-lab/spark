#!/usr/bin/env bash

set -euo pipefail

profile="${1:-default}"

case "$profile" in
	ros|wsl|default)
		shift || true
		;;
	*)
		echo "Usage: $0 {ros|wsl|default} [docker-compose-args...]" >&2
		exit 1
		;;
esac

UID_UID=$(id -u):$(id -g) docker compose --profile "$profile" up "$@"
