#!/usr/bin/env python3
"""
Standalone ONNX Export — Isaac Sim 없이 CPU에서 실행 가능
체크포인트에서 actor 네트워크 구조를 자동으로 읽어 ONNX로 export

Usage:
  python3 scripts/export_onnx_standalone.py \
      --checkpoint logs/rsl_rl/unitree_go2_competition/2026-05-23_04-36-36/model_19200.pt \
      --obs-dim 45 \
      --out logs/rsl_rl/unitree_go2_competition/2026-05-23_04-36-36/exported/policy.onnx
"""

import argparse
import os
from pathlib import Path

import torch
import torch.nn as nn


class ActorMLP(nn.Module):
    """RSL-RL Actor를 재현 (ELU activation, 마지막 레이어는 linear)"""

    def __init__(self, obs_dim: int, hidden_dims: list[int], action_dim: int):
        super().__init__()
        layers = []
        in_dim = obs_dim
        for h in hidden_dims:
            layers.append(nn.Linear(in_dim, h))
            layers.append(nn.ELU())
            in_dim = h
        layers.append(nn.Linear(in_dim, action_dim))
        self.mlp = nn.Sequential(*layers)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.mlp(obs)


def build_from_checkpoint(ckpt_path: str) -> tuple[ActorMLP, int, int, list[int]]:
    """체크포인트에서 레이어 크기를 자동 파악해 ActorMLP 복원"""
    ckpt = torch.load(ckpt_path, map_location="cpu")

    actor_sd = ckpt["actor_state_dict"]

    # 레이어 크기 파악
    # mlp.0.weight → (h0, obs_dim)
    # mlp.2.weight → (h1, h0)
    # ...
    # mlp.N.weight → (action_dim, h_last)
    linear_layers = sorted(
        [(k, v) for k, v in actor_sd.items() if k.endswith(".weight") and k.startswith("mlp.")],
        key=lambda x: int(x[0].split(".")[1]),
    )

    obs_dim = linear_layers[0][1].shape[1]
    action_dim = linear_layers[-1][1].shape[0]
    hidden_dims = [v.shape[0] for _, v in linear_layers[:-1]]

    print(f"  obs_dim    : {obs_dim}")
    print(f"  hidden_dims: {hidden_dims}")
    print(f"  action_dim : {action_dim}")
    print(f"  iter       : {ckpt.get('iter', 'N/A')}")

    # 모델 생성 & 가중치 로드
    # RSL-RL MLP: Linear → ELU → Linear → ELU → ... → Linear
    # Sequential 인덱스: 0=Linear, 1=ELU, 2=Linear, ...
    # state_dict key: mlp.0, mlp.2, mlp.4 ... (짝수 = Linear)
    model = ActorMLP(obs_dim, hidden_dims, action_dim)

    # 직접 mlp Sequential에 로드 (키 형식이 동일)
    mlp_sd = {k: v for k, v in actor_sd.items() if k.startswith("mlp.")}
    model.load_state_dict(mlp_sd)

    return model, obs_dim, action_dim, hidden_dims


def export_onnx(model: ActorMLP, obs_dim: int, out_path: str):
    model.eval()
    dummy = torch.zeros(1, obs_dim)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    torch.onnx.export(
        model,
        dummy,
        out_path,
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
        opset_version=17,
    )
    print(f"  → ONNX saved: {out_path}")

    # 검증
    import onnxruntime as ort
    sess = ort.InferenceSession(out_path, providers=["CPUExecutionProvider"])
    import numpy as np
    out = sess.run(None, {"obs": np.zeros((1, obs_dim), dtype=np.float32)})
    print(f"  → ONNX inference OK: output shape = {out[0].shape}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", required=True, help="Path to model_XXXXX.pt")
    parser.add_argument(
        "--out",
        default=None,
        help="Output ONNX path (default: <checkpoint_dir>/exported/policy.onnx)",
    )
    args = parser.parse_args()

    ckpt_path = args.checkpoint
    out_path = args.out
    if out_path is None:
        ckpt_dir = Path(ckpt_path).parent
        out_path = str(ckpt_dir / "exported" / "policy.onnx")

    print(f"[export] checkpoint: {ckpt_path}")
    model, obs_dim, action_dim, hidden_dims = build_from_checkpoint(ckpt_path)

    print(f"[export] exporting to ONNX...")
    export_onnx(model, obs_dim, out_path)

    print("[export] Done ✓")


if __name__ == "__main__":
    main()
