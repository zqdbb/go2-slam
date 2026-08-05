#!/usr/bin/env bash
set -euo pipefail

TARGET="${1:-${HOME}/go2_roughnav_ws/src/go2_roughnav}"
OVERLAY_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -d "${TARGET}" ]]; then
  echo "[ERROR] target repo not found: ${TARGET}" >&2
  exit 1
fi

echo "[INFO] applying overlay to ${TARGET}"

mkdir -p "${TARGET}/go2_roughnav" "${TARGET}/launch" "${TARGET}/config"
cp -v "${OVERLAY_DIR}/go2_roughnav/"*.py "${TARGET}/go2_roughnav/"
cp -v "${OVERLAY_DIR}/launch/"*.launch.py "${TARGET}/launch/"
cp -v "${OVERLAY_DIR}/config/"*.yaml "${TARGET}/config/"

python3 - "${TARGET}/setup.py" <<'PY'
from __future__ import annotations

import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()

def insert_after(text: str, marker: str, line: str) -> str:
    if line.strip() in text:
        return text
    idx = text.find(marker)
    if idx < 0:
        raise SystemExit(f"marker not found: {marker}")
    end = text.find("\n", idx)
    return text[: end + 1] + line + "\n" + text[end + 1 :]

text = insert_after(
    text,
    '"config/trg_ros2_params_isaac.yaml",',
    '                "config/rl_interface.yaml",',
)
text = insert_after(
    text,
    '"config/fastlio_real_go2.yaml",',
    '                "config/fastlio_go2_hw.yaml",',
)
text = insert_after(
    text,
    '"launch/04_full_isaac_pipeline.launch.py",',
    '                "launch/05_full_go2_rl_pipeline.launch.py",',
)
text = insert_after(
    text,
    '"path_to_cmd_vel = go2_roughnav.path_to_cmd_vel:main",',
    '            "height_scan_bridge = go2_roughnav.height_scan_bridge:main",',
)
path.write_text(text)
print(f"[INFO] patched {path}")
PY

TARGET_PARENT="$(dirname "${TARGET}")"
if [[ "$(basename "${TARGET_PARENT}")" == "src" ]]; then
  WS_DIR="$(dirname "${TARGET_PARENT}")"
else
  WS_DIR="${TARGET_PARENT}"
fi
echo "[INFO] overlay applied. Rebuild with: cd ${WS_DIR} && colcon build --symlink-install"
