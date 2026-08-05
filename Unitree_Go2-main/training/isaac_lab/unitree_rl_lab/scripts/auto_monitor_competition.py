#!/usr/bin/env python3
"""
ICROS 2025 Competition Training Auto-Monitor
컨테이너 내부에서 실행 (docker exec -d)

기능:
  1. 매 5000 iter: 학습 상태 분석 리포트
  2. Phase 자동 전환 (커리큘럼 강제 진행):
       Phase1 (0-10k):   max_init_terrain_level=0  (순수 커리큘럼)
       Phase2 (10k-20k): max_init_terrain_level=5  (중간 지형 강제)
       Phase3 (20k-30k): max_init_terrain_level=9  (모든 지형 강제)
  3. Terrain level 정체 감지 → Phase 조기 전환
  4. 보상 저하 감지 → 경고 + 필요시 개입
  5. 체크포인트에서 재시작 (이어 학습)

실행:
  docker exec -d isaac-lab-template /isaac-sim/kit/python/bin/python3 \
    /workspace/unitree_rl_lab/scripts/auto_monitor_competition.py
"""

import glob
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime

# ─────────────────────────────────────────────────────────────────────────────
# 설정
# ─────────────────────────────────────────────────────────────────────────────
WORKSPACE        = "/workspace/unitree_rl_lab"
TRAIN_LOG        = "/tmp/train_competition.log"
MONITOR_LOG      = "/tmp/auto_monitor_competition.log"
OVERRIDE_FILE    = "/tmp/training_override.json"
CHECKPOINT_BASE  = f"{WORKSPACE}/logs/rsl_rl/unitree_go2_competition"
PYTHON_SH        = "/isaac-sim/python.sh"   # 반드시 이 wrapper 사용 (환경 설정 포함)
TOTAL_ITER       = 30000
ANALYSIS_EVERY   = 5000    # iter마다 분석
CHECK_SLEEP      = 120     # 2분마다 로그 확인 (iter당 ~2.5s → 5000iter ≈ 3.5시간)

# Phase 정의 — iter 도달 시 max_init_terrain_level 변경 + 재시작
PHASES = [
    {"name": "Phase1-Curriculum",     "starts_at": 0,     "max_init_level": 0},
    {"name": "Phase2-MediumExposure", "starts_at": 10000, "max_init_level": 5},
    {"name": "Phase3-HardExposure",   "starts_at": 20000, "max_init_level": 9},
]

# 정체 판정 임계값
PLATEAU_TERRAIN_DELTA = 0.04   # 5000 iter간 terrain_level 변화 < 이 값 → 정체
PLATEAU_REWARD_DELTA  = -0.15  # vel_reward 변화 < 이 값 → 보상 저하


# ─────────────────────────────────────────────────────────────────────────────
# 유틸 — 로깅
# ─────────────────────────────────────────────────────────────────────────────
def log(msg: str, level: str = "INFO"):
    """MONITOR_LOG 파일 + stdout 동시 출력."""
    ts = datetime.now().strftime("%H:%M:%S")
    line = f"[{ts}][{level}] {msg}"
    with open(MONITOR_LOG, "a") as f:
        f.write(line + "\n")
    print(line, flush=True)


# ─────────────────────────────────────────────────────────────────────────────
# 유틸 — 로그 파싱
# ─────────────────────────────────────────────────────────────────────────────
def count_iters(content: str) -> int:
    """'Iteration time:' 줄 수 → 완료된 iteration 수."""
    return len(re.findall(r"Iteration time:", content))


def extract_metric_series(content: str, key: str) -> list[float]:
    """content에서 특정 메트릭의 모든 값 추출 (시간 순)."""
    pattern = re.escape(key) + r":\s+([+-]?\d+\.?\d*(?:e[+-]?\d+)?)"
    return [float(x) for x in re.findall(pattern, content)]


def parse_latest_metrics(content: str) -> dict:
    """마지막 iteration 블록에서 핵심 메트릭 추출."""
    # Iteration time: 을 기준으로 마지막 블록 잘라내기
    last_idx = content.rfind("Iteration time:")
    if last_idx == -1:
        return {}
    # 그 앞의 ~5000자 안에서 메트릭 추출
    block = content[max(0, last_idx - 5000): last_idx + 200]

    def get(pattern: str) -> float | None:
        m = re.search(pattern, block)
        return float(m.group(1)) if m else None

    return {
        "terrain_level":      get(r"Curriculum/terrain_levels:\s+([+-]?\d+\.?\d*)"),
        "lin_vel_cmd_level":  get(r"Curriculum/lin_vel_cmd_levels:\s+([+-]?\d+\.?\d*)"),
        "track_vel_xy":       get(r"Episode_Reward/track_lin_vel_xy:\s+([+-]?\d+\.?\d*)"),
        "termination_pen":    get(r"Episode_Reward/termination_penalty:\s+([+-]?\d+\.?\d*)"),
        "ep_len":             get(r"Mean episode length:\s+(\d+\.?\d*)"),
        "action_std":         get(r"Mean action std:\s+(\d+\.?\d*)"),
        "timeout_rate":       get(r"Episode_Termination/time_out:\s+(\d+\.?\d*)"),
        "bad_orientation":    get(r"Episode_Termination/bad_orientation:\s+(\d+\.?\d*)"),
        "base_contact":       get(r"Episode_Termination/base_contact:\s+(\d+\.?\d*)"),
        "iter_time":          get(r"Iteration time:\s+(\d+\.?\d*)"),
        "eta":                re.search(r"ETA:\s+([\d:]+)", block).group(1)
                              if re.search(r"ETA:\s+([\d:]+)", block) else "?",
    }


# ─────────────────────────────────────────────────────────────────────────────
# 유틸 — 체크포인트
# ─────────────────────────────────────────────────────────────────────────────
def get_latest_checkpoint() -> tuple[str, str] | tuple[None, None]:
    """최신 (run_dir, checkpoint_filename) 반환.
    get_checkpoint_path()는 절대경로가 아닌 파일명만 받으므로 분리해서 반환.
    """
    pattern = f"{CHECKPOINT_BASE}/*/model_*.pt"
    files = glob.glob(pattern)
    if not files:
        return None, None
    # 수정 시간 기준 최신 파일
    files.sort(key=os.path.getmtime, reverse=True)
    latest = files[0]
    run_dir = os.path.basename(os.path.dirname(latest))
    filename = os.path.basename(latest)
    return run_dir, filename


# ─────────────────────────────────────────────────────────────────────────────
# 훈련 관리
# ─────────────────────────────────────────────────────────────────────────────
def write_override(max_init_level: int):
    """override JSON 작성 → env_cfg import 시 읽음."""
    override = {
        "max_init_terrain_level": max_init_level,
        "written_at": datetime.now().isoformat(),
    }
    with open(OVERRIDE_FILE, "w") as f:
        json.dump(override, f, indent=2)
    log(f"Override 작성: max_init_terrain_level={max_init_level}", "ACTION")


def kill_training():
    """현재 학습 프로세스 종료."""
    # train.py를 실행 중인 Python 프로세스만 종료
    result = subprocess.run(
        ["pkill", "-f", "scripts/rsl_rl/train.py"],
        capture_output=True,
    )
    time.sleep(8)  # PhysX 정리 대기
    log("훈련 프로세스 종료", "ACTION")


def start_training(run_dir: str | None, ckpt_name: str | None, max_iter: int, max_init_level: int):
    """훈련 재시작 (컨테이너 내부).
    run_dir: 타임스탬프 디렉토리명 (예: 2026-05-22_10-20-54)
    ckpt_name: 파일명만 (예: model_10000.pt) — get_checkpoint_path 제약
    """
    if run_dir and ckpt_name:
        resume_args = f"--resume --load_run {run_dir} --checkpoint {ckpt_name}"
    else:
        resume_args = ""
    cmd = (
        f"cd {WORKSPACE} && "
        f"nohup /isaac-sim/python.sh scripts/rsl_rl/train.py "
        f"--task Unitree-Go2-Competition --headless "
        f"--max_iterations {max_iter} {resume_args} "
        f">> {TRAIN_LOG} 2>&1 &"
    )
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        close_fds=True,
    )
    log(f"훈련 재시작 (PID group: {proc.pid}, "
        f"checkpoint={run_dir}/{ckpt_name if ckpt_name else 'None'}, "
        f"max_iter={max_iter}, max_init_level={max_init_level})", "ACTION")


def transition_phase(new_phase: dict, current_iter: int):
    """Phase 전환: override 작성 → kill → restart."""
    log("─" * 65, "ACTION")
    log(f"Phase 전환: {new_phase['name']} (max_init_level={new_phase['max_init_level']})", "ACTION")

    run_dir, ckpt_name = get_latest_checkpoint()
    if run_dir:
        log(f"체크포인트: {run_dir}/{ckpt_name}", "ACTION")
    else:
        log("체크포인트 없음 — 처음부터 재시작", "WARN")

    kill_training()
    write_override(new_phase["max_init_level"])
    start_training(run_dir, ckpt_name, TOTAL_ITER, new_phase["max_init_level"])
    time.sleep(30)
    log("Phase 전환 완료", "ACTION")


# ─────────────────────────────────────────────────────────────────────────────
# 분석 리포트
# ─────────────────────────────────────────────────────────────────────────────
def print_report(current_iter: int, metrics: dict,
                 terrain_series: list[float], vel_series: list[float],
                 current_phase: dict) -> dict:
    """5000 iter 분석 리포트 출력. 분석 결과 dict 반환."""

    # 이전 5000 iter 구간의 terrain/vel 변화
    window = 200   # 최근 5000 iter ≈ 약 200개 레코드 (iter당 1개 기준)
    if len(terrain_series) >= window * 2:
        terrain_prev = terrain_series[-window * 2]
        terrain_now  = terrain_series[-1]
    elif len(terrain_series) >= 2:
        terrain_prev = terrain_series[0]
        terrain_now  = terrain_series[-1]
    else:
        terrain_prev = terrain_now = metrics.get("terrain_level", 0) or 0

    if len(vel_series) >= window * 2:
        vel_prev = vel_series[-window * 2]
        vel_now  = vel_series[-1]
    elif len(vel_series) >= 2:
        vel_prev = vel_series[0]
        vel_now  = vel_series[-1]
    else:
        vel_prev = vel_now = metrics.get("track_vel_xy", 0) or 0

    terrain_delta = terrain_now - terrain_prev
    vel_delta     = vel_now - vel_prev

    m = metrics
    sep = "=" * 65

    log(f"\n{sep}", "REPORT")
    log(f"📊  분석 리포트 — Iter {current_iter:,} / {TOTAL_ITER:,}", "REPORT")
    log(f"    Phase: {current_phase['name']}  (max_init_level={current_phase['max_init_level']})", "REPORT")
    log(sep, "REPORT")
    log(f"  🗺  Terrain Level  : {terrain_now:.2f} / 9  (raw 누적 레벨)", "REPORT")
    log(f"      Δ (last 5k)   : {terrain_delta:+.3f}  "
        f"({'⚠️ 정체' if abs(terrain_delta) < PLATEAU_TERRAIN_DELTA else '✅ 진행 중'})", "REPORT")
    log(f"  🏃  Vel Reward     : {vel_now:.4f}  Δ {vel_delta:+.4f}  "
        f"({'⚠️ 저하' if vel_delta < PLATEAU_REWARD_DELTA else '✅ 양호'})", "REPORT")
    log(f"  ⏱   Episode Length : {m.get('ep_len', 0) or 0:.0f} steps", "REPORT")
    log(f"  ✅  Timeout Rate   : {(m.get('timeout_rate', 0) or 0):.1%}  "
        f"(20s 생존 비율)", "REPORT")
    log(f"  ❌  Fall Rate      : bad_orient {(m.get('bad_orientation', 0) or 0):.1%} | "
        f"base_contact {(m.get('base_contact', 0) or 0):.1%}", "REPORT")
    log(f"  🎯  Action Std     : {m.get('action_std', 0) or 0:.3f}  "
        f"(낮을수록 확신 있는 행동)", "REPORT")
    log(f"  ⚡  Iter Speed     : {m.get('iter_time', 0) or 0:.2f}s/iter  ETA: {m.get('eta', '?')}", "REPORT")
    log(sep, "REPORT")

    return {
        "terrain_now":     terrain_now,
        "terrain_delta":   terrain_delta,
        "vel_now":         vel_now,
        "vel_delta":       vel_delta,
        "is_plateau":      abs(terrain_delta) < PLATEAU_TERRAIN_DELTA,
        "is_degrading":    vel_delta < PLATEAU_REWARD_DELTA,
        "timeout_rate":    m.get("timeout_rate", 0) or 0,
    }


# ─────────────────────────────────────────────────────────────────────────────
# 개입 결정
# ─────────────────────────────────────────────────────────────────────────────
def decide_intervention(analysis: dict, current_iter: int,
                        current_phase: dict) -> tuple[bool, str, int]:
    """
    개입 여부를 결정.
    반환: (개입 여부, 이유 설명, 새 max_init_terrain_level)
    """
    terrain    = analysis["terrain_now"]   # raw 값 (0~9 이미 스케일됨)
    is_plateau = analysis["is_plateau"]
    is_degrade = analysis["is_degrading"]
    cur_level  = current_phase["max_init_level"]

    # ─ 케이스 A: 쉬운 구간(level<4) 정체 → 중간 레벨 강제 노출
    if is_plateau and terrain < 4.0 and cur_level < 4:
        new = min(int(terrain) + 2, 5)
        return True, f"Level {terrain:.1f}/9 정체 → max_init={new} 강제", new

    # ─ 케이스 B: 중간 구간(4≤level<7) 정체 → 상위 레벨 강제 노출
    if is_plateau and 4.0 <= terrain < 7.0 and cur_level < 7:
        new = min(int(terrain) + 2, 8)
        return True, f"Level {terrain:.1f}/9 정체 → max_init={new} 강제", new

    # ─ 케이스 C: 어려운 구간(level≥7) 정체 → 수용 가능
    if is_plateau and terrain >= 7.0:
        log(f"고난도 Level {terrain:.1f}/9 정체 — 정상 범위, 계속 학습", "INFO")
        return False, "", cur_level

    # ─ 케이스 D: 보상 저하 — 경고만 (단기 저하는 어려운 지형 탐색 중일 수 있음)
    if is_degrade:
        log(f"보상 저하 감지 (Δvel={analysis['vel_delta']:.4f}) — 5k iter 추가 관찰", "WARN")
        return False, "", cur_level

    return False, "", cur_level


# ─────────────────────────────────────────────────────────────────────────────
# 메인 루프
# ─────────────────────────────────────────────────────────────────────────────
def main():
    log("=" * 65, "INFO")
    log("ICROS 2025 Auto-Monitor v3 시작", "INFO")
    log(f"분석 주기: {ANALYSIS_EVERY} iter  |  총 목표: {TOTAL_ITER} iter", "INFO")
    log(f"Phase 계획:", "INFO")
    for p in PHASES:
        log(f"  iter {p['starts_at']:>6,}: {p['name']}  max_init={p['max_init_level']}", "INFO")
    log("=" * 65, "INFO")

    last_analysis_iter = 0
    no_data_count      = 0
    prev_iter          = 0

    # override 파일로 현재 Phase 복원 (재시작 시 중복 전환 방지)
    current_phase_idx = 0
    if os.path.exists(OVERRIDE_FILE):
        try:
            cur_level = json.load(open(OVERRIDE_FILE)).get("max_init_terrain_level", 0)
            for i, p in enumerate(PHASES):
                if p["max_init_level"] == cur_level:
                    current_phase_idx = i
                    break
            log(f"Override 복원: Phase {current_phase_idx} (max_init_level={cur_level})", "INFO")
        except Exception:
            pass

    while True:
        time.sleep(CHECK_SLEEP)

        # ── 로그 읽기 ────────────────────────────────────────────────────────
        try:
            with open(TRAIN_LOG, "r") as f:
                content = f.read()
        except Exception as e:
            no_data_count += 1
            log(f"로그 읽기 실패 ({no_data_count}회): {e}", "WARN")
            if no_data_count > 30:  # 60분간 로그 없으면 종료
                log("60분간 로그 없음 — 훈련 종료 추정, monitor 종료", "WARN")
                break
            continue

        no_data_count  = 0
        current_iter   = count_iters(content)

        if current_iter == 0:
            log("아직 iteration 없음 — 초기화 중...", "INFO")
            continue

        if current_iter == prev_iter:
            continue   # 진행 없으면 스킵
        prev_iter = current_iter

        # ── Phase 자동 전환 ─────────────────────────────────────────────────
        # 현재 iter에 맞는 Phase 결정
        target_phase_idx = 0
        for i in range(len(PHASES) - 1, -1, -1):
            if current_iter >= PHASES[i]["starts_at"]:
                target_phase_idx = i
                break

        if target_phase_idx > current_phase_idx:
            new_phase = PHASES[target_phase_idx]
            log(f"\niter {current_iter}: Phase 전환 트리거", "ACTION")
            transition_phase(new_phase, current_iter)
            current_phase_idx = target_phase_idx
            last_analysis_iter = current_iter   # 재시작 후 분석 기준 리셋
            continue

        # ── 5000 iter 분석 ──────────────────────────────────────────────────
        if current_iter - last_analysis_iter >= ANALYSIS_EVERY:
            metrics       = parse_latest_metrics(content)
            terrain_ser   = extract_metric_series(content, "Curriculum/terrain_levels")
            vel_ser       = extract_metric_series(content, "Episode_Reward/track_lin_vel_xy")

            analysis = print_report(
                current_iter, metrics,
                terrain_ser, vel_ser,
                PHASES[current_phase_idx],
            )
            last_analysis_iter = current_iter

            # ── 개입 결정 ────────────────────────────────────────────────────
            should, reason, new_level = decide_intervention(
                analysis, current_iter, PHASES[current_phase_idx])

            if should:
                log(f"\n⚠️ 개입: {reason}", "ACTION")
                run_dir, ckpt_name = get_latest_checkpoint()
                if run_dir:
                    kill_training()
                    write_override(new_level)
                    start_training(run_dir, ckpt_name, TOTAL_ITER, new_level)
                    PHASES[current_phase_idx] = dict(
                        PHASES[current_phase_idx], max_init_level=new_level)
                    time.sleep(30)
                else:
                    log("체크포인트 없음 — 개입 불가", "WARN")

        # ── 완료 확인 ────────────────────────────────────────────────────────
        if current_iter >= TOTAL_ITER:
            run_dir, ckpt_name = get_latest_checkpoint()
            log(f"\n🏁 훈련 완료! (iter {current_iter:,})", "DONE")
            log(f"최신 체크포인트: {run_dir}/{ckpt_name}", "DONE")
            log("Auto-Monitor 종료", "DONE")
            break


if __name__ == "__main__":
    main()
