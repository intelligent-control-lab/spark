#!/usr/bin/env bash

set -euo pipefail

usage (){
	echo "Usage: $0 {ros|wsl|default} <spark-run-file>" >&2
}

profile="${1:-default}"

# check for container variant
case "$profile" in
	ros|wsl|default)
		shift || true
		;;
	*)
		usage
		echo 'Must specify a container variant, e.g., "default"'
		exit 1
		;;
esac

# check for run file
if [[ $# -eq 0 ]]; then
	usage
	echo "Must specify a spark run file"
	exit 1
fi

docker compose run --remove-orphans spark-${profile} bash -ic "source /root/.bashrc && python $1"
