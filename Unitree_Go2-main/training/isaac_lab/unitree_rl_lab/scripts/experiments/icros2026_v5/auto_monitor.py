#!/usr/bin/env python3
"""V5 Auto Monitor — Checkpoint-based Phase Transitions (V4 log-count 버그 수정).

## V4에서 발견된 버그
  - log-count ("Iteration time" 문자열 수)로 phase 전환 → RSL-RL log buffer로 실제 iter의 ~20%만 캡처
  - Phase 3이 자동으로 전환 안 됨 → 수동 개입 필요
  - TOTAL_ITER=30000인데 model_30000 미저장 → ckpt_iter=32999에서 완료 오검출

## V5 수정 사항
  - 체크포인트 파일 번호 기반 phase 전환 (model_XXXXX.pt의 숫자가 정확한 진행 상태)
  - TOTAL_ITER=40000, Phase3 start=20000 (ckpt 기반)
  - 모든 write_override에 dr_phase 포함 (V3 버그 재발 방지)
  - 완료 조건: ckpt_iter >= TOTAL_ITER - 1 (off-by-one 허용)
"""

import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# ── 설정 ──────────────────────────────────────────────────────────────────────
CONTAINER    = "isaac-lab-template"
TASK         = "Unitree-Go2-ICROS2026-V5"
NUM_ENVS     = 16384
LOG_FILE     = "/tmp/train_icros2026_v5.log"
LOG_DIR      = "/workspace/logs/rsl_rl/unitree_go2_icros2026_v5"
OVERRIDE_PATH = "/tmp/training_override.json"
STATE_FILE   = "/tmp/monitor_state_icros2026_v5.json"
DONE_FILE    = "/tmp/monitor_done_icros2026_v5"

TRAIN_SCRIPT = "/workspace/unitree_rl_lab/scripts/experiments/icros2026_v5/train.py"

TOTAL_ITER     = 40000
CHECK_INTERVAL = 60    # 60초마다 체크
# save_interval=50, iter≈5s → 체크포인트 간격 250s
# STALL_TIMEOUT = 250s × 4배 = 1000s (약 16분)
STALL_TIMEOUT  = 1000
# ckpt=0: Isaac Sim 초기화(~6분) + 첫 체크포인트(50iter×5s≈4분) = 10분
# → 절대 재시작 안 함: ckpt=0이면 is_running() 확인만, hang 판정 없음
INIT_TIMEOUT   = 99999  # 사실상 무한대: ckpt=0일 때 절대 재시작 안 함

# ── Phase 정의 (ckpt_iter 기반) ───────────────────────────────────────────────
# 중요: start_ckpt는 체크포인트 파일 번호 기준 (log count 아님!)
PHASES = [
    {
        "name": "Phase1",
        "start_ckpt": 0,
        "max_init_level": 0,
        "lin_vel_x_max": 1.0,
        "lin_vel_x_min": 0.0,
        "dr_phase": 1,
        "desc": "기초 보행 + 평지 (DR 최소)",
    },
    {
        "name": "Phase2",
        "start_ckpt": 10000,
        "max_init_level": 4,
        "lin_vel_x_max": 1.5,
        "lin_vel_x_min": 0.0,
        "dr_phase": 2,
        "desc": "중간 지형 + DR 강화",
    },
    {
        "name": "Phase3",
        "start_ckpt": 20000,
        "max_init_level": 7,
        "lin_vel_x_max": 2.0,
        "lin_vel_x_min": -0.5,
        "dr_phase": 3,
        "desc": "고난도 지형 + 최대 DR + 30k iter",
    },
]


# ─────────────────────────────────────────────────────────────────────────────
def log(msg: str):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{ts}] {msg}", flush=True)


def docker_exec(cmd: str) -> str:
    """컨테이너 내부 명령 실행 후 stdout 반환."""
    result = subprocess.run(
        ["docker", "exec", CONTAINER, "bash", "-c", cmd],
        capture_output=True, text=True, timeout=30
    )
    return result.stdout.strip()


def get_latest_checkpoint() -> tuple[str, int]:
    """전체 V5 로그에서 가장 높은 번호 체크포인트 반환.

    Returns:
        (path, iter_num) — 없으면 ("", 0)
    """
    out = docker_exec(
        f"find {LOG_DIR} -maxdepth 2 -name 'model_*.pt' 2>/dev/null"
        r" | awk -F'model_' '{print $2, $0}'"
        r" | sed 's/\.pt / /'"
        " | sort -k1 -n | tail -1"
    )
    if not out:
        return "", 0
    parts = out.split()
    if len(parts) < 2:
        return "", 0
    try:
        iter_num = int(parts[0])
        path = " ".join(parts[1:])
        return path, iter_num
    except ValueError:
        return "", 0


def get_training_pids() -> list[int]:
    """컨테이너에서 V5 train.py 관련 프로세스 PID 목록을 반환."""
    out = docker_exec(
        "ps -eo pid=,args= "
        f"| grep -F '{TRAIN_SCRIPT}' "
        f"| grep -F '{TASK}' "
        "| grep -v grep "
        "| awk '{print $1}'"
    )
    pids: list[int] = []
    for token in out.split():
        try:
            pids.append(int(token))
        except ValueError:
            pass
    return pids


def get_training_python_pids() -> list[int]:
    """실제 V5 학습 Python 프로세스 PID 목록을 반환."""
    out = docker_exec(
        "ps -eo pid=,args= "
        f"| grep -F '{TRAIN_SCRIPT}' "
        f"| grep -F '{TASK}' "
        "| grep -E 'python|python3' "
        "| grep -v '/isaac-sim/python.sh' "
        "| grep -v grep "
        "| awk '{print $1}'"
    )
    pids: list[int] = []
    for token in out.split():
        try:
            pids.append(int(token))
        except ValueError:
            pass
    return pids


def is_training_running() -> bool:
    """컨테이너에서 V5 train.py 프로세스가 실행 중인지 확인."""
    return len(get_training_pids()) > 0


def stop_training(force: bool = False):
    """현재 V5 학습 프로세스만 종료."""
    pids = get_training_pids()
    if not pids:
        log("  종료할 V5 학습 프로세스 없음")
        return

    pid_text = " ".join(str(pid) for pid in pids)
    log(f"  V5 학습 종료 요청: pids={pid_text}, force={force}")
    docker_exec(f"kill -INT {pid_text} 2>/dev/null || true")
    time.sleep(20)

    remaining = get_training_pids()
    if force or remaining:
        if remaining:
            rem_text = " ".join(str(pid) for pid in remaining)
            log(f"  남은 V5 학습 강제 종료: pids={rem_text}")
            docker_exec(f"kill -9 {rem_text} 2>/dev/null || true")
            time.sleep(5)


def count_monitor_processes() -> int:
    """host supervisor가 참고할 수 있도록 현재 monitor 수를 반환."""
    out = subprocess.run(
        ["bash", "-lc", "pgrep -f 'scripts/experiments/icros2026_v5/auto_monitor.py' | wc -l"],
        capture_output=True, text=True
    ).stdout.strip()
    try:
        return int(out)
    except ValueError:
        return 1


def write_override(max_init: int, vel_max: float, vel_min: float, dr_phase: int):
    """override JSON을 컨테이너에 쓰기.

    주의: V3/V4 버그 재발 방지 — 항상 dr_phase 포함!
    """
    override = {
        "max_init_terrain_level": max_init,
        "lin_vel_x_max": vel_max,
        "lin_vel_x_min": vel_min,
        "dr_phase": dr_phase,
    }
    json_str = json.dumps(override)
    docker_exec(f"echo '{json_str}' > {OVERRIDE_PATH}")
    log(f"  Override 작성: {override}")


def start_training(ckpt_path: str = "") -> bool:
    """학습 시작 (ckpt_path 있으면 resume)."""
    if ckpt_path:
        run_dir  = docker_exec(f"basename $(dirname {ckpt_path})")
        ckpt_file = os.path.basename(ckpt_path)
        m = re.search(r"model_(\d+)\.pt$", ckpt_file)
        ckpt_iter = int(m.group(1)) if m else 0
        remaining = max(TOTAL_ITER - ckpt_iter, 1)
        cmd = (
            f"nohup /isaac-sim/python.sh {TRAIN_SCRIPT}"
            f" --task {TASK} --headless --num_envs {NUM_ENVS}"
            f" --resume --load_run {run_dir} --checkpoint {ckpt_file}"
            f" --max_iterations {remaining}"
            f" > {LOG_FILE} 2>&1"
        )
        log(f"학습 재시작: run={run_dir} ckpt={ckpt_file} remaining_iter={remaining}")
    else:
        cmd = (
            f"nohup /isaac-sim/python.sh {TRAIN_SCRIPT}"
            f" --task {TASK} --headless --num_envs {NUM_ENVS}"
            f" --max_iterations {TOTAL_ITER}"
            f" > {LOG_FILE} 2>&1"
        )
        log(f"학습 신규 시작: max_iterations={TOTAL_ITER}")

    docker_exec(f"bash -c '{cmd} &'")
    time.sleep(5)
    return True


def load_state() -> dict:
    if os.path.exists(STATE_FILE):
        try:
            with open(STATE_FILE) as f:
                return json.load(f)
        except Exception:
            pass
    return {"current_phase_idx": 0, "last_ckpt_iter": 0, "stall_count": 0}


def save_state(state: dict):
    with open(STATE_FILE, "w") as f:
        json.dump(state, f, indent=2)


def mark_done():
    """호스트/컨테이너 양쪽에 완료 플래그를 남긴다."""
    Path(DONE_FILE).touch()
    docker_exec(f"touch {DONE_FILE}")


def get_current_phase(ckpt_iter: int) -> tuple[int, dict]:
    """ckpt_iter에 해당하는 Phase 반환."""
    phase_idx = 0
    for i, ph in enumerate(PHASES):
        if ckpt_iter >= ph["start_ckpt"]:
            phase_idx = i
    return phase_idx, PHASES[phase_idx]


# ─────────────────────────────────────────────────────────────────────────────
def main():
    log("=" * 60)
    log("V5 Auto Monitor 시작")
    log(f"  Task: {TASK} | NumEnvs: {NUM_ENVS} | TotalIter: {TOTAL_ITER}")
    log(f"  Phase 전환: checkpoint 번호 기반 (log-count 버그 수정)")
    log("=" * 60)

    state = load_state()
    log(f"상태 로드: phase_idx={state['current_phase_idx']}, last_ckpt={state['last_ckpt_iter']}")

    # 초기 override 설정 (현재 phase에 맞게)
    ckpt_path, ckpt_iter = get_latest_checkpoint()
    cur_phase_idx, cur_phase = get_current_phase(max(ckpt_iter, state["last_ckpt_iter"]))
    write_override(
        cur_phase["max_init_level"],
        cur_phase["lin_vel_x_max"],
        cur_phase["lin_vel_x_min"],
        cur_phase["dr_phase"],
    )
    log(f"초기 Phase: {cur_phase['name']} (ckpt={ckpt_iter})")

    # 학습이 안 돌고 있으면 시작
    if not is_training_running():
        if ckpt_iter >= TOTAL_ITER - 1:
            log(f"✅ 이미 완료 (ckpt={ckpt_iter}). Monitor 종료.")
            mark_done()
            return
        log("학습 프로세스 없음 → 시작")
        start_training(ckpt_path)

    state["current_phase_idx"] = cur_phase_idx
    stall_count = 0

    while True:
        time.sleep(CHECK_INTERVAL)

        ckpt_path, ckpt_iter = get_latest_checkpoint()

        # ── 완료 체크 (off-by-one 허용: model_49999 or model_50000) ──────────
        if ckpt_iter >= TOTAL_ITER - 1:
            log(f"🎉 V5 baseline 완료! (ckpt_iter={ckpt_iter}) 학습 정지 후 Monitor 종료.")
            stop_training(force=False)
            mark_done()
            save_state({"current_phase_idx": len(PHASES)-1, "last_ckpt_iter": ckpt_iter, "stall_count": 0})
            return

        # ── 중복 학습 방지 ───────────────────────────────────────────────────
        python_pids = get_training_python_pids()
        if len(python_pids) > 1:
            all_pids = get_training_pids()
            log(f"🔴 V5 학습 Python 프로세스 중복 감지! python_pids={python_pids}, all_pids={all_pids} → 최신 ckpt에서 단일 재시작")
            cur_phase_idx, cur_phase = get_current_phase(ckpt_iter)
            write_override(
                cur_phase["max_init_level"],
                cur_phase["lin_vel_x_max"],
                cur_phase["lin_vel_x_min"],
                cur_phase["dr_phase"],
            )
            stop_training(force=True)
            start_training(ckpt_path)
            state["current_phase_idx"] = cur_phase_idx
            state["last_ckpt_iter"] = ckpt_iter
            stall_count = 0
            save_state(state)
            continue

        # ── Phase 전환 체크 (checkpoint 번호 기반) ────────────────────────────
        new_phase_idx, new_phase = get_current_phase(ckpt_iter)
        if new_phase_idx != state["current_phase_idx"]:
            log(f"🔄 Phase 전환: {PHASES[state['current_phase_idx']]['name']} → {new_phase['name']}")
            log(f"   ckpt_iter={ckpt_iter}, start_ckpt={new_phase['start_ckpt']}")
            log(f"   {new_phase['desc']}")
            write_override(
                new_phase["max_init_level"],
                new_phase["lin_vel_x_max"],
                new_phase["lin_vel_x_min"],
                new_phase["dr_phase"],
            )
            state["current_phase_idx"] = new_phase_idx

            # override 반영을 위해 학습 재시작
            log("  Phase 전환 → 현재 checkpoint에서 재시작")
            stop_training(force=False)
            start_training(ckpt_path)
            stall_count = 0
            state["last_ckpt_iter"] = ckpt_iter
            save_state(state)
            continue

        # ── Hang/Crash 감지 ───────────────────────────────────────────────────
        # ckpt=0이면 Isaac Sim 초기화 중 (INIT_TIMEOUT 적용)
        # ckpt>0이면 실제 학습 정체 (STALL_TIMEOUT 적용)
        effective_timeout = INIT_TIMEOUT if ckpt_iter == 0 else STALL_TIMEOUT

        if is_training_running():
            if ckpt_iter > state["last_ckpt_iter"]:
                stall_count = 0
                log(f"✅ {new_phase['name']} | ckpt={ckpt_iter} | "
                    f"phase_progress={ckpt_iter - new_phase['start_ckpt']}/{TOTAL_ITER - new_phase['start_ckpt']}")
            else:
                stall_count += 1
                stall_secs = stall_count * CHECK_INTERVAL
                label = "초기화 대기" if ckpt_iter == 0 else "Iter 정체"
                log(f"⏳ {label} (ckpt={ckpt_iter}, {stall_secs}s / {effective_timeout}s)")

                if stall_secs >= effective_timeout:
                    log("🔴 HANG 감지! 재시작")
                    stop_training(force=True)
                    cur_ph = PHASES[state["current_phase_idx"]]
                    write_override(  # V3/V4 버그 재발 방지: dr_phase 항상 포함
                        cur_ph["max_init_level"],
                        cur_ph["lin_vel_x_max"],
                        cur_ph["lin_vel_x_min"],
                        cur_ph["dr_phase"],
                    )
                    start_training(ckpt_path)
                    stall_count = 0
        else:
            # 프로세스 없음 → 크래시
            log(f"🔴 학습 프로세스 없음! (ckpt={ckpt_iter}) 재시작")
            cur_ph = PHASES[state["current_phase_idx"]]
            write_override(  # V3/V4 버그 재발 방지: dr_phase 항상 포함
                cur_ph["max_init_level"],
                cur_ph["lin_vel_x_max"],
                cur_ph["lin_vel_x_min"],
                cur_ph["dr_phase"],
            )
            start_training(ckpt_path)
            stall_count = 0

        state["last_ckpt_iter"] = ckpt_iter
        save_state(state)


if __name__ == "__main__":
    main()
