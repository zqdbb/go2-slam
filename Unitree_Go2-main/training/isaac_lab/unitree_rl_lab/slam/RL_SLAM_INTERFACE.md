# RL-SLAM Interface Contract

This document fixes the runtime contract between the SLAM/planning pipeline and
the Go2 RL locomotion policy.

## Decision

The competition task is obstacle traversal, not simple obstacle avoidance. The
RL policy must therefore receive both a velocity command and local terrain
geometry.

Required inputs to the RL deploy node:

| Topic | Type | Owner | Meaning |
|------|------|-------|---------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | SLAM/planning | Desired planar body velocity |
| `/rl/height_scan` | `std_msgs/msg/Float32MultiArray` | RL adapter or SLAM | 273-dim local terrain height grid |
| Go2 lowstate | Unitree SDK2 | RL | IMU, joint positions, joint velocities |

The V5 policy input is:

```text
proprio_history(225) + height_scan(273) = 498
```

`/cmd_vel` alone is not enough for the V5 policy and is not enough as the main
strategy for stairs, holes, and high rough terrain.

## `/cmd_vel` Semantics

`geometry_msgs/msg/Twist` contains two 3D vectors:

```text
linear.x   forward/backward velocity, m/s
linear.y   lateral velocity, m/s
linear.z   vertical velocity, unused for Go2 locomotion

angular.x  roll rate, unused
angular.y  pitch rate, unused
angular.z  yaw rate, rad/s
```

Important:

- `angular.z` is rotation around the vertical z axis. It is not upward motion.
- `angular.z` must be treated as yaw rate in `rad/s`.
- Do not overload `angular.z` as an absolute target heading.
- If TRG/planner has an absolute target heading, convert heading error to yaw
  rate before publishing `/cmd_vel`.

Recommended initial limits for real Go2 testing:

```text
linear.x   [-0.10, 0.50] m/s
linear.y   [-0.25, 0.25] m/s
angular.z  [-0.80, 0.80] rad/s
```

For stairs and gaps, the planner should slow down before the obstacle, but the
actual foot clearance must come from the RL policy using `/rl/height_scan`.

## `/rl/height_scan` Semantics

The height scan must match the V5 training observation as closely as possible:

```text
type:     std_msgs/msg/Float32MultiArray
length:   273
layout:   21 x 13 grid, flattened row-major with x first
x range:  [-1.0, 1.0] m relative to base
y range:  [-0.6, 0.6] m relative to base
res:      0.1 m
value:    terrain height relative to robot base z
clip:     [-1.0, 1.0]
empty:    -1.0 initially; may be improved after real bag validation
rate:     20 Hz minimum, 50 Hz preferred
```

The current RL-side implementation is:

```bash
python3 scripts/ros2/height_scan_bridge.py \
  --cloud-topic /cloud_registered \
  --odom-topic /Odometry \
  --out-topic /rl/height_scan
```

It converts FAST-LIO point cloud and odometry into the V5 273-dim height scan.

## Recommended Pipeline

```text
Go2 LiDAR/IMU
  -> FAST-LIO2                     -> /cloud_registered + /Odometry
  -> TRG-planner                   -> /path
  -> path_to_cmd_vel               -> /cmd_vel
  -> height_scan_bridge            -> /rl/height_scan
  -> RL deploy node                -> joint targets
  -> Unitree SDK2 low-level control
```

SLAM/planning may keep ownership of FAST-LIO, GICP, TRG, and `/cmd_vel`.
RL owns the final policy input assembly and low-level joint command generation.

