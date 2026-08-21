import json
import math
import os
import sys
from collections import deque

import mujoco
import mujoco.viewer
import numpy as np
import torch

# MuJoCo runs a real-time control loop; avoid OpenMP thread oversubscription.
torch.set_num_threads(1)
torch.set_num_interop_threads(1)


SIM_DT = 0.005
POLICY_DT = 0.02
POLICY_DECIMATION = int(POLICY_DT / SIM_DT)
KP = 25.0
KD = 0.6
ACTION_SCALE = 0.25
ACTION_EMA_ALPHA = 0.3
SCALE_ANG_VEL = 0.2
SCALE_JOINT_VEL = 0.05
HISTORY_LEN = 5

# Isaac Lab order: FL/FR/RL/RR grouped by joint type.
# MuJoCo order: FR, FL, RR, RL with hip/thigh/calf per leg.
IL2MJ = np.array([3, 0, 9, 6, 4, 1, 10, 7, 5, 2, 11, 8])
MJ2IL = np.array([1, 5, 9, 0, 4, 8, 3, 7, 11, 2, 6, 10])
DEFAULT_JOINT_POS_IL = np.array([
    0.1, -0.1, 0.1, -0.1,
    0.8, 0.8, 1.0, 1.0,
    -1.5, -1.5, -1.5, -1.5,
], dtype=np.float64)
DEFAULT_JOINT_POS_MJ = DEFAULT_JOINT_POS_IL[MJ2IL]
INITIAL_STAND_POS_MJ = np.array([
    0.0, 0.8, -1.5,
    0.0, 0.8, -1.5,
    0.0, 1.0, -1.5,
    0.0, 1.0, -1.5,
], dtype=np.float64)
MJ_JOINT_NAMES = [
    "FR_hip", "FR_thigh", "FR_calf",
    "FL_hip", "FL_thigh", "FL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
]

SCAN_SIZE_X = 2.0
SCAN_SIZE_Y = 1.2
SCAN_RES = 0.1
SCAN_NX = round(SCAN_SIZE_X / SCAN_RES) + 1
SCAN_NY = round(SCAN_SIZE_Y / SCAN_RES) + 1
SCAN_DIM = SCAN_NX * SCAN_NY
HEIGHT_SCAN_OFFSET = 0.43
SCAN_OFFSETS = np.array([
    (x, y)
    for x in np.linspace(-SCAN_SIZE_X / 2, SCAN_SIZE_X / 2, SCAN_NX)
    for y in np.linspace(-SCAN_SIZE_Y / 2, SCAN_SIZE_Y / 2, SCAN_NY)
], dtype=np.float64)


class Actor(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.mlp = torch.nn.Sequential(
            torch.nn.Linear(498, 512),
            torch.nn.ELU(),
            torch.nn.Linear(512, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 12),
        )

    def forward(self, obs):
        return self.mlp(obs)


def load_actor(checkpoint_path):
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    state = checkpoint["actor_state_dict"]
    actor = Actor()
    actor.load_state_dict({k: v for k, v in state.items() if k.startswith("mlp.")})
    actor.eval()
    return actor


model_path = sys.argv[1]
policy_path = sys.argv[2]
model = mujoco.MjModel.from_xml_path(model_path)
model.opt.timestep = SIM_DT
data = mujoco.MjData(model)
actor = load_actor(policy_path)
base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
if base_id < 0:
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
if base_id < 0:
    raise RuntimeError("MuJoCo model has neither base_link nor base body")

viewer = None
if "--viewer" in sys.argv[3:] or os.environ.get("MUJOCO_VIEWER", "0") == "1":
    try:
        viewer = mujoco.viewer.launch_passive(model, data)
    except Exception:
        viewer = None


def get_joint_states():
    pos = np.array([data.sensor(f"{name}_pos").data[0] for name in MJ_JOINT_NAMES])
    vel = np.array([data.sensor(f"{name}_vel").data[0] for name in MJ_JOINT_NAMES])
    return pos, vel


def apply_pd_action(action_il):
    pos_mj, vel_mj = get_joint_states()
    target_il = action_il * ACTION_SCALE + DEFAULT_JOINT_POS_IL
    target_mj = target_il[MJ2IL]
    torque_mj = KP * (target_mj - pos_mj) - KD * vel_mj
    for actuator_id in range(model.nu):
        lo, hi = model.actuator_ctrlrange[actuator_id]
        data.ctrl[actuator_id] = float(np.clip(torque_mj[actuator_id], lo, hi))


def quat_to_rot_mat(quat_wxyz):
    flat = np.zeros(9, dtype=np.float64)
    mujoco.mju_quat2Mat(flat, quat_wxyz)
    return flat.reshape(3, 3)


def projected_gravity(quat_wxyz):
    return quat_to_rot_mat(quat_wxyz).T @ np.array([0.0, 0.0, -1.0])


def compute_height_scan():
    robot_pos = data.qpos[:3].copy()
    robot_quat = data.qpos[3:7].copy()
    yaw = math.atan2(
        2.0 * (robot_quat[0] * robot_quat[3] + robot_quat[1] * robot_quat[2]),
        1.0 - 2.0 * (robot_quat[2] ** 2 + robot_quat[3] ** 2),
    )
    rotation = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    heights = np.zeros(SCAN_DIM, dtype=np.float32)
    down = np.array([0.0, 0.0, -1.0])
    for index, (dx, dy) in enumerate(SCAN_OFFSETS):
        world_offset = rotation @ np.array([dx, dy, 0.0])
        world_pos = robot_pos + world_offset
        ray_origin = np.array([world_pos[0], world_pos[1], robot_pos[2] + 2.0])
        geom_id = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(model, data, ray_origin, down, None, 1, -1, geom_id)
        if 0.0 <= distance < 3.0:
            ground_z = ray_origin[2] - distance
            height = robot_pos[2] - ground_z - HEIGHT_SCAN_OFFSET
        else:
            height = -1.0
        heights[index] = np.clip(height, -1.0, 1.0)
    return heights


last_action = np.zeros(12, dtype=np.float32)
smooth_action = np.zeros(12, dtype=np.float32)
proprio_history = deque(
    [np.zeros(45, dtype=np.float32) for _ in range(HISTORY_LEN)],
    maxlen=HISTORY_LEN,
)


def compute_policy_action(command):
    global last_action, smooth_action
    pos_mj, vel_mj = get_joint_states()
    pos_il = pos_mj[IL2MJ]
    vel_il = vel_mj[IL2MJ]
    obs_45 = np.concatenate([
        data.sensor("imu_gyro").data.copy() * SCALE_ANG_VEL,
        projected_gravity(data.sensor("imu_quat").data.copy()),
        command.astype(np.float64),
        pos_il - DEFAULT_JOINT_POS_IL,
        vel_il * SCALE_JOINT_VEL,
        last_action,
    ]).astype(np.float32)
    proprio_history.append(obs_45.copy())
    observation = np.concatenate([
        np.concatenate(list(proprio_history)),
        compute_height_scan(),
    ]).astype(np.float32)
    with torch.inference_mode():
        raw_action = actor(torch.from_numpy(observation).unsqueeze(0))[0].numpy()
    raw_action = np.clip(raw_action, -1.0, 1.0)
    smooth_action = (
        ACTION_EMA_ALPHA * smooth_action
        + (1.0 - ACTION_EMA_ALPHA) * raw_action
    )
    last_action = raw_action.copy()
    return smooth_action


def reset_robot():
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = [0.0, 0.0, 0.445]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for index, joint_name in enumerate(MJ_JOINT_NAMES):
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{joint_name}_joint"
        )
        data.qpos[int(model.jnt_qposadr[joint_id])] = INITIAL_STAND_POS_MJ[index]
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_forward(model, data)

    # Place all four foot spheres on the local static floor before warm-up.
    static_groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
    base_heights = []
    for foot_name in ("FL_foot", "FR_foot", "RL_foot", "RR_foot"):
        foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, foot_name)
        foot_pos = data.xpos[foot_id].copy()
        ray_origin = foot_pos + np.array([0.0, 0.0, 1.0])
        geom_id = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(
            model, data, ray_origin, np.array([0.0, 0.0, -1.0]),
            static_groups, 1, -1, geom_id,
        )
        if distance > 0.0:
            ground_z = ray_origin[2] - distance
            base_heights.append(data.qpos[2] + ground_z + 0.022 - foot_pos[2])
    if base_heights:
        data.qpos[2] = max(base_heights)
    mujoco.mj_forward(model, data)

    # A stronger stand-up controller is used only during initialization. The
    # learned controller takes over after contacts have settled.
    for _ in range(400):
        pos_mj, vel_mj = get_joint_states()
        stand_torque = 55.0 * (INITIAL_STAND_POS_MJ - pos_mj) - 2.0 * vel_mj
        for actuator_id in range(model.nu):
            lo, hi = model.actuator_ctrlrange[actuator_id]
            data.ctrl[actuator_id] = float(np.clip(stand_torque[actuator_id], lo, hi))
        mujoco.mj_step(model, data)

    # The map safety plane is at z=-0.1. Re-level after contact settling so
    # the policy starts from the same upright state used during training.
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_forward(model, data)


reset_robot()


def get_state():
    pos = data.xpos[base_id].tolist()
    quat = data.xquat[base_id].tolist()
    vel = data.qvel[0:6].tolist()
    joints = []
    for joint_id in range(model.njnt):
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            continue
        joints.append({
            "name": model.joint(joint_id).name,
            "pos": float(data.qpos[int(model.jnt_qposadr[joint_id])]),
            "vel": float(data.qvel[int(model.jnt_dofadr[joint_id])]),
        })
    return {
        "time": float(data.time),
        "pos": pos,
        "quat": quat,
        "vel": vel,
        "joints": joints,
    }


def do_lidar(n_h, n_v, vfov_deg, max_range, lidar_rate):
    state = get_state()
    rotation = quat_to_rot_mat(np.asarray(state["quat"], dtype=float))
    sensor_offset = np.array([0.18, 0.0, 0.18])
    origin = np.asarray(state["pos"], dtype=float) + rotation @ sensor_offset
    vfov = math.radians(vfov_deg)
    points = []
    period = 1.0 / lidar_rate
    for ring, vertical_angle in enumerate(np.linspace(-vfov / 2, vfov / 2, n_v)):
        cv, sv = math.cos(vertical_angle), math.sin(vertical_angle)
        for horizontal_index, horizontal_angle in enumerate(
            np.linspace(-math.pi, math.pi, n_h, endpoint=False)
        ):
            local_direction = np.array([
                cv * math.cos(horizontal_angle),
                cv * math.sin(horizontal_angle),
                sv,
            ])
            direction = rotation @ local_direction
            geom_id = np.array([-1], dtype=np.int32)
            distance = mujoco.mj_ray(
                model, data, origin, direction, None, 1, base_id, geom_id
            )
            if 0.05 < distance < max_range:
                point = local_direction * distance
                points.append([
                    float(point[0]), float(point[1]), float(point[2]),
                    max(0.0, 1.0 - distance / max_range), ring,
                    period * (ring * n_h + horizontal_index) / max(1, n_h * n_v - 1),
                ])
    return points


cmd_vx = cmd_vy = cmd_yaw = 0.0
last_lidar_t = -1.0
viewer_sync_count = 0
lidar_n_h = 180
lidar_n_v = 16
lidar_vfov = 30.0
lidar_max_range = 10.0
lidar_rate = 10.0

sys.stdout.write(json.dumps({
    "ready": True,
    "controller": "v5_model_40000",
    "observation_dim": 498,
}) + "\n")
sys.stdout.flush()

for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    try:
        request = json.loads(line)
    except Exception:
        continue
    action = request.get("action", "step")
    if action == "config_lidar":
        lidar_n_h = request.get("n_h", lidar_n_h)
        lidar_n_v = request.get("n_v", lidar_n_v)
        lidar_vfov = request.get("vfov", lidar_vfov)
        lidar_max_range = request.get("max_range", lidar_max_range)
        lidar_rate = request.get("rate", lidar_rate)
        sys.stdout.write(json.dumps({"ok": True}) + "\n")
        sys.stdout.flush()
        continue
    if action not in ("step", "cmd_vel"):
        continue

    cmd_vx = float(np.clip(request.get("vx", cmd_vx), -1.0, 2.0))
    cmd_vy = float(np.clip(request.get("vy", cmd_vy), -0.5, 0.5))
    cmd_yaw = float(np.clip(request.get("yaw", cmd_yaw), -2.0, 2.0))
    command = np.array([cmd_vx, cmd_vy, cmd_yaw], dtype=np.float32)
    policy_action = compute_policy_action(command)
    apply_pd_action(policy_action)

    if viewer is not None:
        with viewer.lock():
            for _ in range(POLICY_DECIMATION):
                mujoco.mj_step(model, data)
        viewer_sync_count += 1
        if viewer_sync_count % 2 == 0:
            viewer.sync()
    else:
        for _ in range(POLICY_DECIMATION):
            mujoco.mj_step(model, data)

    state = get_state()
    response = {"state": state, "controller": "v5_model_40000"}
    if state["time"] - last_lidar_t >= 1.0 / lidar_rate:
        response["points"] = do_lidar(
            lidar_n_h, lidar_n_v, lidar_vfov, lidar_max_range, lidar_rate
        )
        last_lidar_t = state["time"]
    sys.stdout.write(json.dumps(response) + "\n")
    sys.stdout.flush()
