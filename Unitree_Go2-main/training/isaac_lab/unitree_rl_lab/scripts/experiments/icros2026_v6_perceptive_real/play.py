# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
from importlib.metadata import version

from isaaclab.app import AppLauncher

# local imports — cli_args.py is in scripts/rsl_rl/; add that dir to path
import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.parent / "rsl_rl"))
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")
parser.add_argument("--lin_vel_x", type=float, default=None, help="Fixed forward velocity command (m/s). If set, overrides random commands.")
parser.add_argument("--lin_vel_y", type=float, default=None, help="Fixed lateral velocity command (m/s).")
parser.add_argument("--ang_vel_z", type=float, default=None, help="Fixed yaw velocity command (rad/s).")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import time
import torch

from rsl_rl.runners import OnPolicyRunner

import isaaclab_tasks  # noqa: F401
from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict
try:
    from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint
except ImportError:
    get_published_pretrained_checkpoint = None
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx, handle_deprecated_rsl_rl_cfg
from isaaclab_tasks.utils import get_checkpoint_path

import unitree_rl_lab.tasks  # noqa: F401
from unitree_rl_lab.utils.parser_cfg import parse_env_cfg


def main():
    """Play with RSL-RL agent."""
    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
        entry_point_key="play_env_cfg_entry_point",
    )
    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", args_cli.task)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # apply compatibility fix for rsl_rl version differences (same as train.py)
    from importlib import metadata
    installed_version = metadata.version("rsl-rl-lib")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, installed_version)
    # load previously trained model
    if not hasattr(agent_cfg, "class_name") or agent_cfg.class_name == "OnPolicyRunner":
        runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        from rsl_rl.runners import DistillationRunner

        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    runner.load(resume_path)

    # obtain the trained policy for inference
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # extract the neural network module
    # we do this in a try-except to maintain backwards compatibility.
    try:
        # new rsl_rl API (>= 4.0)
        policy_nn = runner.alg.actor
    except AttributeError:
        try:
            # version 2.3
            policy_nn = runner.alg.policy
        except AttributeError:
            # version 2.2 and below
            policy_nn = runner.alg.actor_critic

    # extract the normalizer
    if hasattr(policy_nn, "actor_obs_normalizer"):
        normalizer = policy_nn.actor_obs_normalizer
    elif hasattr(policy_nn, "student_obs_normalizer"):
        normalizer = policy_nn.student_obs_normalizer
    else:
        normalizer = None

    # export policy to onnx/jit
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    try:
        export_policy_as_jit(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.pt")
        export_policy_as_onnx(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.onnx")
    except Exception as e:
        print(f"[WARNING] Policy export skipped: {e}")

    dt = env.unwrapped.step_dt

    # fixed command override setup
    use_fixed_cmd = any(v is not None for v in [args_cli.lin_vel_x, args_cli.lin_vel_y, args_cli.ang_vel_z])
    if use_fixed_cmd:
        fixed_vx = args_cli.lin_vel_x if args_cli.lin_vel_x is not None else 0.0
        fixed_vy = args_cli.lin_vel_y if args_cli.lin_vel_y is not None else 0.0
        fixed_wz = args_cli.ang_vel_z if args_cli.ang_vel_z is not None else 0.0
        print(f"[INFO] Fixed velocity command: vx={fixed_vx}, vy={fixed_vy}, wz={fixed_wz}")

    def _extract_policy_obs(obs):
        if isinstance(obs, dict):
            return obs.get("policy", obs)
        if hasattr(obs, "keys") and "policy" in obs.keys():
            return obs["policy"]
        return obs

    def _put_policy_obs(obs_orig, policy_obs):
        if isinstance(obs_orig, dict) or hasattr(obs_orig, "keys"):
            obs_orig["policy"] = policy_obs
            return obs_orig
        return policy_obs

    def _get_cmd_slots(policy_obs):
        obs_dim = policy_obs.shape[-1]
        if obs_dim in (225, 498):
            return [(k * 45 + 6, k * 45 + 9) for k in range(5)]
        return [(6, 9)]

    def override_cmd_and_obs(obs):
        """Override command manager tensor and every command slot in policy obs."""
        if use_fixed_cmd:
            cmd = env.unwrapped.command_manager.get_command("base_velocity")
            cmd[:, 0] = fixed_vx
            cmd[:, 1] = fixed_vy
            cmd[:, 2] = fixed_wz
            policy_obs = _extract_policy_obs(obs).clone()
            for start, end in _get_cmd_slots(policy_obs):
                policy_obs[:, start:end] = torch.tensor(
                    [fixed_vx, fixed_vy, fixed_wz],
                    device=policy_obs.device,
                    dtype=policy_obs.dtype,
                )
            obs = _put_policy_obs(obs, policy_obs)
        return obs

    # reset environment
    try:
        obs, _ = env.get_observations()
    except (TypeError, ValueError):
        obs = env.get_observations()
    obs = override_cmd_and_obs(obs)
    timestep = 0
    # simulate environment
    while simulation_app.is_running():
        start_time = time.time()
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)
            # env stepping
            obs, _, _, _ = env.step(actions)
        obs = override_cmd_and_obs(obs)
        if args_cli.video:
            timestep += 1
            # Exit the play loop after recording one video
            if timestep == args_cli.video_length:
                break

        # time delay for real-time evaluation
        sleep_time = dt - (time.time() - start_time)
        if args_cli.real_time and sleep_time > 0:
            time.sleep(sleep_time)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
