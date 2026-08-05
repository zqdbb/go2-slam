#!/usr/bin/env bash
# Optional path: seed V6 from the latest completed V5 checkpoint and start V6.
#
# The default V6 path is scratch training via start_scratch_after_v5.sh. Use this
# only if we explicitly want to compare V5-seeded fine-tuning against scratch.

set -euo pipefail

CONTAINER="isaac-lab-template"
V5_TASK="Unitree-Go2-ICROS2026-V5"
V5_SCRIPT="/workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/train.py"
V5_LOG_DIR="/workspace/logs/rsl_rl/unitree_go2_icros2026_v5"
V6_LOG_DIR="/workspace/logs/rsl_rl/unitree_go2_icros2026_v6_perceptive_real"

REPO="/home/nuri/unitree_rl_lab"
V6_WATCHDOG="${REPO}/scripts/experiments/icros2026_v6_perceptive_real/watchdog_host.sh"
V6_DONE="/tmp/monitor_done_icros2026_v6_perceptive_real"
V6_STATE="/tmp/monitor_state_icros2026_v6_perceptive_real.json"
V6_MONITOR_LOG="/tmp/auto_monitor_icros2026_v6_perceptive_real.log"
V6_SUPERVISOR_LOG="/tmp/auto_monitor_icros2026_v6_perceptive_real_supervisor.log"

v5_running="$(
  docker exec "${CONTAINER}" bash -lc \
    "ps -eo args= | grep -F '${V5_SCRIPT}' | grep -F '${V5_TASK}' | grep -v grep | wc -l"
)"

if [ "${v5_running}" != "0" ]; then
  echo "V5 training is still running; wait until the V5 40k monitor stops it."
  exit 1
fi

latest_line="$(
  docker exec "${CONTAINER}" bash -lc \
    "find ${V5_LOG_DIR} -maxdepth 2 -name 'model_*.pt' 2>/dev/null \
      | awk -F'model_' '{print \\$2, \\$0}' \
      | sed 's/\\.pt / /' \
      | sort -k1 -n \
      | tail -1"
)"

if [ -z "${latest_line}" ]; then
  echo "No V5 checkpoint found."
  exit 1
fi

ckpt_iter="$(printf '%s\n' "${latest_line}" | awk '{print $1}')"
ckpt_path="$(printf '%s\n' "${latest_line}" | cut -d' ' -f2-)"

if [ "${ckpt_iter}" -lt 40000 ]; then
  echo "Latest V5 checkpoint is model_${ckpt_iter}.pt; wait for model_40000.pt or newer."
  exit 1
fi

seed_run="v5_seed_${ckpt_iter}"
docker exec "${CONTAINER}" bash -lc "
  mkdir -p '${V6_LOG_DIR}/${seed_run}' &&
  cp '${ckpt_path}' '${V6_LOG_DIR}/${seed_run}/model_${ckpt_iter}.pt' &&
  rm -f /tmp/monitor_done_icros2026_v6_perceptive_real /tmp/monitor_state_icros2026_v6_perceptive_real.json
"

rm -f "${V6_DONE}" "${V6_STATE}" "${V6_MONITOR_LOG}" "${V6_SUPERVISOR_LOG}"

setsid "${V6_WATCHDOG}" >> "${V6_SUPERVISOR_LOG}" 2>&1 < /dev/null &

echo "Seeded V6 from ${ckpt_path}"
echo "Started V6 watchdog pid=$!"
