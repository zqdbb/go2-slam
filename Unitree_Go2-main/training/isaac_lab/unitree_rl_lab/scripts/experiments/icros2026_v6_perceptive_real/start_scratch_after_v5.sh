#!/usr/bin/env bash
# Wait for the V5 40k baseline to finish, then start V6 from scratch.
#
# This is the default V6 path. V5 is kept only as a baseline/eval artifact
# because its gait appears plateaued around terrain level 5.5.

set -euo pipefail

CONTAINER="isaac-lab-template"
V5_TASK="Unitree-Go2-ICROS2026-V5"
V5_SCRIPT="/workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/train.py"
V5_LOG_DIR="/workspace/logs/rsl_rl/unitree_go2_icros2026_v5"
V6_LOG_DIR="/workspace/logs/rsl_rl/unitree_go2_icros2026_v6_perceptive_real"

REPO="/home/nuri/unitree_rl_lab"
V6_WATCHDOG="${REPO}/scripts/experiments/icros2026_v6_perceptive_real/watchdog_host.sh"
LOG_FILE="/tmp/start_v6_scratch_after_v5.log"
V6_DONE="/tmp/monitor_done_icros2026_v6_perceptive_real"
V6_STATE="/tmp/monitor_state_icros2026_v6_perceptive_real.json"
V6_MONITOR_LOG="/tmp/auto_monitor_icros2026_v6_perceptive_real.log"
V6_SUPERVISOR_LOG="/tmp/auto_monitor_icros2026_v6_perceptive_real_supervisor.log"

log() {
  printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >> "${LOG_FILE}"
}

latest_v5_ckpt_iter() {
  docker exec "${CONTAINER}" bash -lc "
    find '${V5_LOG_DIR}' -maxdepth 2 -name 'model_*.pt' 2>/dev/null \
      | sed 's/.*model_//;s/\\.pt//' \
      | sort -n \
      | tail -1
  "
}

v5_is_running() {
  docker exec "${CONTAINER}" bash -lc "
    ps -eo args= \
      | grep -F '${V5_SCRIPT}' \
      | grep -F '${V5_TASK}' \
      | grep -v grep \
      | wc -l
  "
}

while true; do
  v5_running="$(v5_is_running)"
  ckpt_iter="$(latest_v5_ckpt_iter)"
  ckpt_iter="${ckpt_iter:-0}"

  if [ "${v5_running}" = "0" ] && [ "${ckpt_iter}" -ge 40000 ]; then
    log "V5 baseline complete: model_${ckpt_iter}.pt"
    break
  fi

  log "waiting for V5 baseline: running=${v5_running}, latest_ckpt=${ckpt_iter}"
  sleep 120
done

existing_v6="$(
  docker exec "${CONTAINER}" bash -lc "
    find '${V6_LOG_DIR}' -maxdepth 2 -name 'model_*.pt' 2>/dev/null | head -1
  "
)"

if [ -n "${existing_v6}" ]; then
  log "V6 checkpoint already exists; refusing scratch start: ${existing_v6}"
  exit 1
fi

docker exec "${CONTAINER}" bash -lc "
  rm -f /tmp/monitor_done_icros2026_v6_perceptive_real \
        /tmp/monitor_state_icros2026_v6_perceptive_real.json
"
rm -f "${V6_DONE}" "${V6_STATE}" "${V6_MONITOR_LOG}" "${V6_SUPERVISOR_LOG}"

setsid "${V6_WATCHDOG}" >> "${V6_SUPERVISOR_LOG}" 2>&1 < /dev/null &
log "started V6 scratch watchdog pid=$!"
