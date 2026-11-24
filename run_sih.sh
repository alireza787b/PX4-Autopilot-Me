#!/usr/bin/env bash
set -euo pipefail

root=$(cd "$(dirname "$0")" && pwd)
variant=$1
scenario=$2
label=${3:-$variant-$scenario}
restart_delay=${4:-550}
binary_dirs=${SITL_BINARY_DIRS:-"$HOME/.cache/pb-tailsitter/pitch-handoff-pr"}
artifact="$binary_dirs/$variant"
run="$root/runs/$label"
name="px4-pr29954-$label"
image=ghcr.io/px4/px4-dev-ros2-gazebo@sha256:4e7bb18e46f8007d3e601e25597f84117ef14d282df10999e2dc0a9fe7d4e1f2

case "$variant" in stock|candidate) ;; *) exit 2 ;; esac
case "$scenario" in nominal) mission=generic ;; delayed) mission=stale_longer ;; *) exit 2 ;; esac
[[ -x "$artifact/bin/px4" ]]
mkdir -p "$root/runs"
mkdir "$run"
mkdir -p "$run/output/fs/etc/logging"
cp "$root/logger_topics.txt" "$run/output/fs/etc/logging/"
sha256sum "$artifact/bin/px4" > "$run/binary.sha256"

docker run -d --rm --name "$name" --network none --user "$(id -u):$(id -g)" \
    -v "$artifact:/clean:ro" -v "$run/output:/output" -v "$root/flight_driver.py:/regression.py:ro" \
    -e HOME=/output -e PX4_SYS_AUTOSTART=10042 -e PX4_SIM_MODEL=sihsim_xvert \
    -e PX4_SIMULATOR=sihsim -e PX4_SIM_SPEED_FACTOR=1 -e PX4_PARAM_COM_RC_IN_MODE=4 \
    -e PX4_PARAM_SDLOG_PROFILE=0 -e PX4_PARAM_SDLOG_MODE=0 \
    --entrypoint bash "$image" -c 'cd /output && exec /clean/bin/px4 -d -i 0 -w /output /clean/etc' \
    > "$run/container-id.txt"
cleanup() {
    docker logs "$name" > "$run/px4.log" 2>&1 || true
    docker stop -t 4 "$name" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker exec -e MAVLINK20=1 -e SIH_SIMULATION_ONLY=1 -e PB_SIM_MISSION="$mission" \
    -e SIH_FW_RESTART_DELAY_MS="$restart_delay" "$name" python3 /regression.py \
    > "$run/mission.log" 2>&1
docker exec "$name" bash -c 'cd /output && /clean/bin/px4-logger stop' > "$run/logger-close.log" 2>&1
grep -q '"event": "completed"' "$run/output/events.jsonl"
