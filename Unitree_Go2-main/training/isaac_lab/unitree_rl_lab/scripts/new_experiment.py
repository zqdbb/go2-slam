#!/usr/bin/env python3
"""unitree_rl_lab 새 실험 자동 생성 도구.

새 환경/정책/설정을 실험할 때 이 스크립트를 먼저 실행하세요.
실험 파일 구조, gym 등록, README, EXPERIMENTS.md 업데이트를 자동으로 처리합니다.

사용법:
    python scripts/new_experiment.py <name>
    python scripts/new_experiment.py <name> --base icros2025 --desc "설명" --env-id Unitree-Go2-X

예시:
    python scripts/new_experiment.py v2_curriculum
    python scripts/new_experiment.py distillation --base icros2025 --desc "Teacher-Student distillation"
    python scripts/new_experiment.py v3_anticurriculum --env-id Unitree-Go2-AntiCurriculum

생성 항목:
    scripts/experiments/<name>/
        play.py         ← <base>/play.py 복사
        train.py        ← <base>/train.py 복사
        README.md       ← 자동 생성 (TODO 포함)

    source/.../go2/experiments/<name>/
        __init__.py
        env_cfg.py      ← 베이스 환경 상속 템플릿
        terrains/
            __init__.py
            custom_terrains.py

    go2/__init__.py     ← gym.register() 자동 추가
    EXPERIMENTS.md      ← 실험 목록 자동 업데이트
"""

from __future__ import annotations

import argparse
import pathlib
import re
import shutil
import sys
from datetime import date

# ─── 경로 설정 ────────────────────────────────────────────────────────────────
REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

SCRIPTS_EXP = REPO_ROOT / "scripts" / "experiments"
GO2_EXP = (
    REPO_ROOT
    / "source" / "unitree_rl_lab" / "unitree_rl_lab"
    / "tasks" / "locomotion" / "robots" / "go2" / "experiments"
)
GO2_INIT = GO2_EXP.parent / "__init__.py"
EXPERIMENTS_MD = REPO_ROOT / "EXPERIMENTS.md"


# ─── 헬퍼 ─────────────────────────────────────────────────────────────────────
def die(msg: str) -> None:
    print(f"❌  {msg}", file=sys.stderr)
    sys.exit(1)


def validate_name(name: str) -> None:
    if not re.fullmatch(r"[a-z0-9][a-z0-9_]*", name):
        die(f"이름은 소문자·숫자·_ 만 허용됩니다 (시작은 소문자/숫자). 받은 값: '{name}'")
    if name == "_template":
        die("'_template'은 예약된 이름입니다.")


def make_env_id(name: str) -> str:
    """v2_curriculum → Unitree-Go2-V2Curriculum"""
    parts = re.split(r"[_\-]", name)
    return "Unitree-Go2-" + "".join(p.capitalize() for p in parts)


# ─── 생성할 파일 내용 ──────────────────────────────────────────────────────────
def _scripts_readme(name: str, env_id: str, desc: str, base: str) -> str:
    today = date.today().isoformat()
    return f"""\
# Experiment: {name}

## 개요
- **설명**: {desc or "(TODO: 실험 목적 작성)"}
- **환경 ID**: `{env_id}`
- **생성일**: {today}
- **기반 실험**: `{base}`
- **상태**: 🔄 진행 중

---

## 파일 구성

| 파일 | 위치 | 설명 |
|------|------|------|
| `play.py` | `scripts/experiments/{name}/` | 커스텀 play 스크립트 |
| `train.py` | `scripts/experiments/{name}/` | 커스텀 train 스크립트 |
| `env_cfg.py` | `source/.../go2/experiments/{name}/` | 환경 설정 |
| `terrains/` | `source/.../go2/experiments/{name}/` | 커스텀 지형 (선택) |

---

## 실행 방법

```bash
# 학습 (Docker 내부)
docker exec -it isaac-lab /isaac-sim/python.sh \\
  /workspace/unitree_rl_lab/scripts/experiments/{name}/train.py \\
  --task {env_id} --headless

# 플레이
docker exec -it isaac-lab /isaac-sim/python.sh \\
  /workspace/unitree_rl_lab/scripts/experiments/{name}/play.py \\
  --task {env_id} --num_envs 8

# 고정 속도 명령으로 플레이
docker exec -it isaac-lab /isaac-sim/python.sh \\
  /workspace/unitree_rl_lab/scripts/experiments/{name}/play.py \\
  --task {env_id} --lin_vel_x 1.0 --num_envs 8
```

---

## 원본 대비 변경 사항

### env_cfg.py
- [ ] TODO: 무엇을, 왜 변경했는지 기록

### play.py / train.py
- 기반: `scripts/experiments/{base}/` 복사
- [ ] TODO: 추가 수정 사항 기록

---

## 학습 결과

| Iter | terrain_level | Vel Reward | 비고 |
|------|--------------|-----------|------|
| - | - | - | 학습 전 |

**최종 체크포인트**: (학습 후 기록)
"""


def _env_init_py(name: str, env_id: str) -> str:
    return f'''\
"""{name} experiment — environment package.

Registered gym environment: {env_id}
"""
'''


def _env_cfg_py(name: str, env_id: str, desc: str) -> str:
    return f'''\
"""{env_id} environment for Unitree Go2.

{desc or "TODO: 실험 목적과 주요 변경 사항 설명 작성"}

기반: velocity_env_cfg.py (원본 Go2 속도 추종 환경)

커스텀 지형을 사용하려면:
  1. terrains/custom_terrains.py 에 terrain 함수/Cfg 구현
  2. 아래 주석 처리된 import 해제
  3. COMPETITION_TERRAIN_CFG 스타일로 TerrainGeneratorCfg 정의
  4. RobotSceneCfg.terrain 오버라이드
"""
from __future__ import annotations

from isaaclab.utils import configclass

# 기반 환경 — 필요에 따라 icros2025 env_cfg로 교체 가능
from unitree_rl_lab.tasks.locomotion.robots.go2.velocity_env_cfg import (
    RobotEnvCfg as _BaseEnvCfg,
)

# 커스텀 지형이 필요하면 주석 해제
# from unitree_rl_lab.tasks.locomotion.robots.go2.experiments.{name}.terrains import (
#     MyCustomTerrainCfg,
# )

# ---------------------------------------------------------------------------
# 📝 여기에 커스텀 설정 클래스/변수 추가
#    예: 보상 함수 오버라이드, 커스텀 지형 구성, DR 설정 등
# ---------------------------------------------------------------------------


@configclass
class RobotEnvCfg(_BaseEnvCfg):
    """{name} 학습 환경 (train).

    오버라이드 예시:
        def __post_init__(self):
            super().__post_init__()
            self.rewards.track_lin_vel_xy.weight = 3.0
            self.scene.num_envs = 8192
    """

    # TODO: 변경할 항목을 여기에 오버라이드
    # 예:
    #   scene = RobotSceneCfg(num_envs=8192, env_spacing=2.5)
    #   rewards = RewardsCfg()  # 커스텀 보상


@configclass
class RobotPlayEnvCfg(RobotEnvCfg):
    """{name} 플레이 환경 (소규모, GUI 확인용)."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = 8
        self.episode_length_s = 20.0
'''


def _terrains_init_py() -> str:
    return '''\
"""Custom terrains for this experiment.

구현 후 아래 주석 해제하여 export하세요.

예시:
    from .custom_terrains import MyTerrainCfg, my_terrain_fn
"""
# from .custom_terrains import MyTerrainCfg, my_terrain_fn
'''


def _terrains_custom_py(name: str) -> str:
    return f'''\
"""Custom terrain generators for {name} experiment.

참고: experiments/icros2025/terrains/custom_terrains.py
IsaacLab HfTerrainBaseCfg 문서: https://isaac-sim.github.io/IsaacLab

구현 방법:
  1. terrain 함수 정의: (difficulty: float, cfg: MyCfg) -> np.ndarray
  2. Cfg 클래스 정의: HfTerrainBaseCfg 상속, function 필드 설정
  3. terrains/__init__.py 에서 export
  4. env_cfg.py 에서 TerrainGeneratorCfg에 포함
"""
from __future__ import annotations

from dataclasses import MISSING

import numpy as np

from isaaclab.terrains.height_field import HfTerrainBaseCfg
from isaaclab.utils import configclass


# ---------------------------------------------------------------------------
# 📝 여기에 커스텀 terrain 구현
# ---------------------------------------------------------------------------

# 예시 (주석 해제 후 수정):
#
# def flat_terrain(difficulty: float, cfg: FlatTerrainCfg) -> np.ndarray:
#     """단순 평지 (난이도 무관)."""
#     hf = np.zeros((cfg.size[0], cfg.size[1]), dtype=np.int16)
#     return hf
#
# @configclass
# class FlatTerrainCfg(HfTerrainBaseCfg):
#     function = flat_terrain
#     size: tuple[float, float] = MISSING
#     horizontal_scale: float = 0.1
#     vertical_scale: float = 0.005
'''


def _gym_register_block(name: str, env_id: str) -> str:
    return f"""
# [{name}] {env_id}
gym.register(
    id="{env_id}",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={{
        "env_cfg_entry_point": f"{{__name__}}.experiments.{name}.env_cfg:RobotEnvCfg",
        "play_env_cfg_entry_point": f"{{__name__}}.experiments.{name}.env_cfg:RobotPlayEnvCfg",
        "rsl_rl_cfg_entry_point": "unitree_rl_lab.tasks.locomotion.agents.rsl_rl_ppo_cfg:BasePPORunnerCfg",
    }},
)
"""


# ─── EXPERIMENTS.md 업데이트 ──────────────────────────────────────────────────
def _update_experiments_md(name: str, env_id: str, desc: str) -> None:
    text = EXPERIMENTS_MD.read_text(encoding="utf-8")
    new_row = (
        f"| [{name}](scripts/experiments/{name}/README.md)"
        f" | `{env_id}`"
        f" | {desc or '(작성 필요)'}"
        " | 🔄 진행 중 |"
    )
    # <!-- EXPERIMENTS_TABLE_END --> 마커 앞에 삽입
    marker = "<!-- EXPERIMENTS_TABLE_END -->"
    if marker in text:
        text = text.replace(marker, f"{new_row}\n{marker}")
    else:
        # 마커 없으면 파일 끝에 추가
        text = text.rstrip() + f"\n{new_row}\n"
    EXPERIMENTS_MD.write_text(text, encoding="utf-8")


# ─── 메인 ─────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="unitree_rl_lab 새 실험 자동 생성",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("name", help="실험 이름 (소문자·숫자·_ 만, 예: v2_curriculum)")
    parser.add_argument(
        "--base", default="icros2025",
        help="기반으로 복사할 실험 이름 (기본: icros2025). play.py/train.py 복사 원본.",
    )
    parser.add_argument("--desc", default="", help="실험 설명 (한 줄)")
    parser.add_argument(
        "--env-id", default="",
        help="gym 환경 ID (기본: Unitree-Go2-<Name> 자동 생성)",
    )
    args = parser.parse_args()

    name = args.name.strip()
    validate_name(name)
    env_id = args.env_id.strip() or make_env_id(name)
    desc = args.desc.strip()
    base = args.base.strip()

    # ── 중복 확인 ──
    s_dir = SCRIPTS_EXP / name
    e_dir = GO2_EXP / name
    if s_dir.exists() or e_dir.exists():
        die(f"실험 '{name}'이 이미 존재합니다. 다른 이름을 사용하세요.")

    print(f"\n🚀  새 실험 생성")
    print(f"    이름     : {name}")
    print(f"    환경 ID  : {env_id}")
    print(f"    설명     : {desc or '(없음)'}")
    print(f"    기반     : {base}")
    print()

    # ── 1. 스크립트 디렉토리 ──
    s_dir.mkdir(parents=True)

    base_scripts = SCRIPTS_EXP / base
    if not base_scripts.exists():
        print(f"    ⚠️  기반 '{base}' 없음 → 빈 스크립트 디렉토리만 생성")
    else:
        for fname in ("play.py", "train.py"):
            src = base_scripts / fname
            if src.exists():
                shutil.copy(src, s_dir / fname)
                print(f"    📄 복사  : scripts/experiments/{name}/{fname}  ← {base}/{fname}")
            else:
                print(f"    ⚠️  {base}/{fname} 없음 (건너뜀)")

    (s_dir / "README.md").write_text(
        _scripts_readme(name, env_id, desc, base), encoding="utf-8"
    )
    print(f"    📄 생성  : scripts/experiments/{name}/README.md")

    # ── 2. 환경 디렉토리 ──
    e_dir.mkdir(parents=True)
    terrains_dir = e_dir / "terrains"
    terrains_dir.mkdir()

    (e_dir / "__init__.py").write_text(_env_init_py(name, env_id), encoding="utf-8")
    (e_dir / "env_cfg.py").write_text(_env_cfg_py(name, env_id, desc), encoding="utf-8")
    (terrains_dir / "__init__.py").write_text(_terrains_init_py(), encoding="utf-8")
    (terrains_dir / "custom_terrains.py").write_text(_terrains_custom_py(name), encoding="utf-8")
    print(f"    📄 생성  : go2/experiments/{name}/env_cfg.py")
    print(f"    📄 생성  : go2/experiments/{name}/terrains/")

    # ── 3. go2/__init__.py gym.register 추가 ──
    init_text = GO2_INIT.read_text(encoding="utf-8")
    GO2_INIT.write_text(init_text + _gym_register_block(name, env_id), encoding="utf-8")
    print(f"    ✏️  업데이트: go2/__init__.py  (+gym.register '{env_id}')")

    # ── 4. EXPERIMENTS.md 업데이트 ──
    _update_experiments_md(name, env_id, desc)
    print(f"    ✏️  업데이트: EXPERIMENTS.md")

    # ── 완료 메시지 ──
    print(f"""
✅  완료! 다음 단계:

  1️⃣  환경 설정 편집:
       source/.../go2/experiments/{name}/env_cfg.py

  2️⃣  (선택) 커스텀 지형 추가:
       source/.../go2/experiments/{name}/terrains/custom_terrains.py

  3️⃣  학습 실행:
       python scripts/experiments/{name}/train.py --task {env_id} --headless

  4️⃣  학습 후 README 업데이트:
       scripts/experiments/{name}/README.md

  환경 등록 확인:
       python scripts/list_envs.py
""")


if __name__ == "__main__":
    main()
