# 项目运行命令

本文档只记录实际运行步骤。默认环境：

    服务器：192.168.100.55
    宿主机项目：~/Desktop/go2/SCAN-Planner-Ros2
    ROS 2 容器：go2_mapping
    MuJoCo 容器：mujoco-go2-mapping
    容器工作空间：/workspace

## 1. 登录服务器并检查容器

    ssh hongzt@192.168.100.55
    docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'

必须确认两个容器都在运行：

    go2_mapping
    mujoco-go2-mapping

如果实际 MuJoCo 容器名称是 \`mujoco-huanghb-pi05-teleop\`，需要将 \`integrated_navigation.launch.py\` 中的 \`mujoco_container\` 改成该名称，或者重新创建为 \`mujoco-go2-mapping\`。

## 2. 启动/恢复 MuJoCo 容器

    docker start mujoco-go2-mapping
    docker exec mujoco-go2-mapping python3.12 -c \
      'import mujoco; print(mujoco.__version__)'

期望输出 \`3.8.1\` 或兼容的 \`3.8.x\`。

## 3. 启动 ROS 2 容器

    docker start go2_mapping

检查工作空间：

    docker exec go2_mapping bash -lc '
      test -f /workspace/mujoco_server.py &&
      test -f /workspace/policies/model_40000.pt &&
      test -f /workspace/maps/go2_mapping_20260820_105700.posegraph &&
      echo workspace-ok
    '

## 4. 编译

完整编译：

    docker exec go2_mapping bash -lc '
      cd /workspace &&
      source /opt/ros/humble/setup.bash &&
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    '

修改规划器、控制器或 RViz 后的快速编译：

    docker exec go2_mapping bash -lc '
      cd /workspace &&
      source /opt/ros/humble/setup.bash &&
      colcon build --packages-select traj_utils scan_planner go2_roughnav --symlink-install
    '

## 5. 启动全部系统

推荐使用项目脚本：

    docker exec go2_mapping bash /workspace/start_mapping.sh

该脚本会启动：

    MuJoCo bridge
    点云转 LaserScan
    odom/body_pose bridge
    SLAM Toolbox localization
    Nav2 planner_server
    SCAN-Planner
    闭环控制器
    卡住脱困节点
    RViz2

如果需要直接运行 launch：

    docker exec -it go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      source /workspace/install/setup.bash &&
      ros2 launch go2_roughnav integrated_navigation.launch.py
    '

## 6. 启动状态检查

新开一个服务器终端执行：

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      source /workspace/install/setup.bash &&
      ros2 lifecycle get /planner_server &&
      ros2 lifecycle get /global_costmap/global_costmap
    '

期望：

    active [3]
    active [3]

检查节点：

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      source /workspace/install/setup.bash &&
      ros2 node list
    '

检查关键话题：

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      source /workspace/install/setup.bash &&
      ros2 topic list | grep -E "/(map|scan|points_raw|Odometry|initial_path|cmd_vel|global_costmap)"
    '

检查频率：

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      timeout 5 ros2 topic hz /points_raw || true
      timeout 5 ros2 topic hz /scan || true
      timeout 5 ros2 topic hz /Odometry || true
    '

正常情况下当前基线大约是：

    /points_raw  5 Hz
    /scan        5 Hz
    /Odometry    50 Hz
    /global_costmap/costmap 约 0.67 Hz

## 7. RViz 操作

集成 launch 会自动启动 RViz2，并加载：

    /workspace/install/go2_roughnav/share/go2_roughnav/rviz/mapping.rviz

RViz 中应看到：

    Map
    Global Path (Nav2)
    Global Costmap (Nav2)
    Robot Footprint (Nav2)
    Local Inflated Obstacles (SCAN)
    Local Trajectory (SCAN)
    Local A-Star (SCAN)
    LaserScan
    RobotModel

发送导航目标：

1. 在 RViz 点击 \`2D Goal Pose\`。
2. 在地图上点击目标位置并拖动确定目标朝向。
3. 观察绿色 \`/initial_path\` 全局路径。
4. 观察局部轨迹和 \`/cmd_vel\`。

检查目标：

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      ros2 topic echo /move_base_simple/goal --once
    '

## 8. 导航运行检查

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      echo CMD_VEL && ros2 topic echo /cmd_vel --once &&
      echo RECOVERY && ros2 topic echo /planning/stuck_recovery_active --once
    '

机器人停止后应看到 \`/cmd_vel\` 接近零；正常导航期间 \`stuck_recovery_active\` 应为 \`false\`。

检查代价地图：

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      ros2 topic info /global_costmap/costmap -v &&
      ros2 topic info /global_costmap/published_footprint -v &&
      ros2 topic info /grid_map/occupancy_inflate -v
    '

## 9. 保存 SLAM 地图

保存前停止机器人运动，然后在容器内执行项目脚本：

    docker exec go2_mapping bash /workspace/save_mapping.sh

检查输出：

    docker exec go2_mapping bash -lc \
      'ls -lh /workspace/maps/*posegraph /workspace/maps/*data /workspace/maps/*.pgm /workspace/maps/*.yaml'

保存后同步并提交：

    rsync -a hongzt@192.168.100.55:~/Desktop/go2/SCAN-Planner-Ros2/maps/ \
      ~/桌面/go2slam/go2_mujoco_slam_scan_navigation/maps/
    cd ~/桌面/go2slam
    git add go2_mujoco_slam_scan_navigation/maps
    git commit -m "Save updated SLAM map"
    git push origin main

## 10. 停止和重启

    docker stop go2_mapping
    docker stop mujoco-go2-mapping

重新启动：

    docker start mujoco-go2-mapping
    docker start go2_mapping
    docker exec go2_mapping bash /workspace/start_mapping.sh

## 11. 常见问题

### RViz 黑屏或没有地图

    echo "$DISPLAY"
    ls -l "$HOME/.Xauthority"
    docker exec go2_mapping bash -lc \
      'ros2 topic echo /map --once'

确认 Fixed Frame 为 \`world\`，并确认 \`/map\`、\`/scan\`、\`/Odometry\` 正在发布。

### MuJoCo bridge 找不到容器

    grep -n mujoco_container \
      src/go2_roughnav/launch/integrated_navigation.launch.py
    docker ps --format '{{.Names}}'

两边必须完全一致。

### Nav2 不 active

    docker exec go2_mapping bash -lc \
      'source /opt/ros/humble/setup.bash &&
       ros2 lifecycle get /planner_server'
    docker logs --tail 200 go2_mapping

### 机器人卡住

    docker exec go2_mapping bash -lc \
      'source /opt/ros/humble/setup.bash &&
       ros2 topic echo /planning/stuck_recovery_active --once'

当前脱困逻辑是在约 2.5 秒内位移小于约 5.5 cm 后触发，最多尝试 3 次。
