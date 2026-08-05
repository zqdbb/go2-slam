#!/usr/bin/env bash
# Host-side supervisor for the V5 auto monitor.
#
# Keeps the Python monitor alive while the user is away. The monitor itself owns
# training restart, phase transitions, duplicate-train cleanup, and 40k baseline capping.

set -u

REPO="/home/nuri/unitree_rl_lab"
MONITOR="${REPO}/scripts/experiments/icros2026_v5/auto_monitor.py"
MONITOR_LOG="/tmp/auto_monitor_icros2026_v5.log"
SUPERVISOR_LOG="/tmp/auto_monitor_icros2026_v5_supervisor.log"
DONE_FILE="/tmp/monitor_done_icros2026_v5"
PATTERN="python3 ${MONITOR}"
SLEEP_SEC=120

log() {
  printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >> "${SUPERVISOR_LOG}"
}

start_monitor() {
  log "starting monitor"
  setsid python3 "${MONITOR}" >> "${MONITOR_LOG}" 2>&1 < /dev/null &
  log "monitor pid=$!"
}

while true; do
  if [ -f "${DONE_FILE}" ]; then
    log "done file exists; supervisor exiting"
    exit 0
  fi

  if ! docker ps --format '{{.Names}}' | grep -qx 'isaac-lab-template'; then
    log "isaac-lab-template is not running; trying docker start"
    docker start isaac-lab-template >> "${SUPERVISOR_LOG}" 2>&1 || true
    sleep 30
  fi

  mapfile -t pids < <(pgrep -f "${PATTERN}" || true)
  if [ "${#pids[@]}" -eq 0 ]; then
    start_monitor
  elif [ "${#pids[@]}" -gt 1 ]; then
    keep="${pids[0]}"
    log "duplicate monitors detected: ${pids[*]}; keeping ${keep}"
    for pid in "${pids[@]:1}"; do
      kill "${pid}" 2>/dev/null || true
    done
  else
    log "monitor alive pid=${pids[0]}"
  fi

  sleep "${SLEEP_SEC}"
done
