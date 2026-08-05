# go2_fast_trg Overlay for V5 RL Integration

This overlay upgrades `SeEEEun/go2_fast_trg` for the V5 Go2 RL policy.

It adds:

- `/cloud_registered + /Odometry -> /rl/height_scan` bridge
- a safer `/path -> /cmd_vel` follower with yaw-rate semantics
- pipeline health checks that include `/rl/height_scan`
- a full launch file for real Go2 + FAST-LIO + TRG + RL terrain input

## Apply

```bash
cd /home/nuri/unitree_rl_lab
bash slam/go2_fast_trg_overlay/apply_overlay.sh \
  ~/go2_roughnav_ws/src/go2_roughnav
```

Then rebuild:

```bash
cd ~/go2_roughnav_ws
colcon build --symlink-install
source install/setup.bash
```

## Run Mapping Only

```bash
ros2 launch go2_roughnav 05_full_go2_rl_pipeline.launch.py \
  launch_trg:=false \
  launch_cmd:=false \
  launch_height_scan:=false \
  fastlio_config_file:=fastlio_go2_hw.yaml
```

## Run Autonomous Traversal

```bash
ros2 launch go2_roughnav 05_full_go2_rl_pipeline.launch.py \
  fastlio_config_file:=fastlio_go2_hw.yaml \
  trg_map:=competition
```

RL-side deploy must subscribe to:

```text
/cmd_vel
/rl/height_scan
Go2 lowstate through Unitree SDK2
```

## Important

`/cmd_vel.angular.z` is yaw rate in rad/s. It is not upward motion and must not
be overloaded as an absolute target heading.
