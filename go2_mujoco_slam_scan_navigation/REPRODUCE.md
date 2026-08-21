# Go2 MuJoCo + SLAM + Nav2 + SCAN-Planner 复现说明

本文档记录当前可运行的集成版本：

    MuJoCo 物理仿真 + RL 步态
            ↓
    点云/激光桥接 → SLAM Toolbox 建图/定位
            ↓
    Nav2 Navfn 全局规划 → SCAN-Planner 局部规划
            ↓
    闭环 cmd_vel 控制 + 卡住脱困 → RViz2

## 1. 版本和镜像

代码仓库：

    https://github.com/zqdbb/go2-slam

项目目录：

    go2_mujoco_slam_scan_navigation/

代码基线提交和标签：

    ccb895d9ad319ea2b64662d0687bdb82e07f88bf
    go2-mujoco-slam-scan-baseline-20260821

GHCR 镜像目标标签（当前服务器 Token 缺少 write:packages，尚未成功上传）：

    ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821
    ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821

go2-humble 用于 go2_mapping；mujoco-huanghb 用于 MuJoCo Python 3.12 / MuJoCo 3.8.1 后端。镜像只保存系统依赖和运行环境，源码、地图和场景文件仍来自本项目目录。

## 2. 获取代码

    mkdir -p ~/Desktop
    git clone https://github.com/zqdbb/go2-slam.git ~/Desktop/go2slam
    cd ~/Desktop/go2slam/go2_mujoco_slam_scan_navigation
    git checkout go2-mujoco-slam-scan-baseline-20260821

如果仓库默认分支配置异常，显式使用 main：

    git fetch origin main
    git switch main

## 3. 拉取 GHCR 镜像

需要 GitHub Personal Access Token。拉取至少需要 read:packages，上传需要额外的 write:packages。

当前状态：代码与本文档已上传 GitHub；两个镜像的目标标签已经确定，但服务器现有 Token 缺少 write:packages，因此下列 docker pull 命令要在镜像完成首次上传后使用。

    echo "$GITHUB_TOKEN" | docker login ghcr.io \
      --username "$GITHUB_USER" --password-stdin

    docker pull ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821
    docker pull ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821

如需使用当前项目的本地镜像名：

    docker tag ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821 go2_humble:latest
    docker tag ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821 mujoco-huanghb:pi05-libero

## 4. 宿主机目录和挂载

服务器当前项目路径：

    /home/hongzt/Desktop/go2/SCAN-Planner-Ros2

容器工作空间：

    /workspace

go2_mapping 将宿主机项目目录 bind mount 到 /workspace，所以源码更新后应重新编译，不必重新制作镜像。

## 5. 创建 ROS 2 容器

下面与当前服务器运行方式等价；用户名、Xauthority 路径按目标机器修改：

    docker run -d \
      --name go2_mapping \
      --gpus all \
      --network host \
      --shm-size=64m \
      -v "$HOME/Desktop/go2/SCAN-Planner-Ros2:/workspace" \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$HOME/.Xauthority:/tmp/.Xauthority" \
      -v /var/run/docker.sock:/var/run/docker.sock \
      ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821 \
      bash /workspace/start_mapping.sh

验证 Nav2：

    docker exec go2_mapping bash -lc \
      'source /opt/ros/humble/setup.bash &&
       source /workspace/install/setup.bash &&
       ros2 lifecycle get /planner_server'

正常状态：

    active [3]

## 6. 创建 MuJoCo 后端容器

当前 MuJoCo 容器保持运行，ROS 桥接节点通过 docker exec 启动 /tmp/mujoco_server.py：

    docker run -d \
      --name mujoco-huanghb-pi05-teleop \
      --shm-size=64m \
      -v /home/huanghb/workspace/lerobot:/workspace/lerobot \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /home/huanghb/.Xauthority:/tmp/.Xauthority-host \
      ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821 \
      tail -f /dev/null

验证 MuJoCo：

    docker exec mujoco-huanghb-pi05-teleop python3.12 -c \
      'import mujoco; print(mujoco.__version__)'

应为 3.8.1 或兼容的 3.8.x。

## 7. 编译工作空间

完整编译：

    docker exec go2_mapping bash -lc '
      cd /workspace &&
      source /opt/ros/humble/setup.bash &&
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release &&
      source install/setup.bash
    '

规划器/RViz 快速增量编译：

    docker exec go2_mapping bash -lc '
      cd /workspace &&
      source /opt/ros/humble/setup.bash &&
      colcon build --packages-select traj_utils scan_planner go2_roughnav --symlink-install
    '

## 8. 启动集成系统

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      source /workspace/install/setup.bash &&
      ros2 launch go2_roughnav integrated_navigation.launch.py
    '

也可以使用：

    docker exec go2_mapping bash /workspace/start_mapping.sh

## 9. 关键话题和 RViz

    docker exec go2_mapping bash -lc '
      source /opt/ros/humble/setup.bash &&
      source /workspace/install/setup.bash &&
      ros2 topic list | grep -E "/(map|scan|Odometry|initial_path|cmd_vel|global_costmap)"
    '

应至少包括：

    /map
    /scan
    /Odometry
    /initial_path
    /cmd_vel
    /global_costmap/costmap
    /global_costmap/published_footprint
    /grid_map/occupancy_inflate

RViz 配置：

    src/go2_roughnav/rviz/mapping.rviz

包含 /map、/initial_path、Nav2 全局代价地图、Nav2 footprint、SCAN 局部膨胀障碍、最新局部轨迹和 A* 候选轨迹。

## 10. 地图文件

当前保存地图：

    maps/go2_mapping_20260820_105700.posegraph
    maps/go2_mapping_20260820_105700.data
    maps/go2_mapping_20260820_105700.pgm
    maps/go2_mapping_20260820_105700.yaml

SLAM Toolbox 恢复时必须同时保留 .posegraph 和 .data；仅有 .pgm 不能恢复完整位姿图。

## 11. 代码更新流程

本机修改项目后：

    cd ~/Desktop/go2slam/go2_mujoco_slam_scan_navigation
    git switch -c fix/execution-collision-safety
    git add .
    git commit -m "Describe the change"
    git push -u origin fix/execution-collision-safety

部署到服务器后重新编译：

    rsync -a --exclude='/.git/' \
      ~/Desktop/go2slam/go2_mujoco_slam_scan_navigation/ \
      hongzt@192.168.100.55:~/Desktop/go2/SCAN-Planner-Ros2/

    ssh hongzt@192.168.100.55 \
      'docker exec go2_mapping bash -lc "cd /workspace &&
       source /opt/ros/humble/setup.bash &&
       colcon build --symlink-install"'

## 12. 当前限制

- build/、install/、log/ 是运行产物，不纳入 Git。
- MuJoCo 镜像很大，GHCR 推送和拉取需要较长时间。
- MuJoCo 的 /workspace/lerobot 是宿主机挂载目录，目标机器必须准备对应路径。
- X11 GUI 需要正确的 DISPLAY 和 Xauthority；无 GUI 环境可以关闭 viewer。
- 当前基线仍保留执行层碰墙风险，后续安全修复应在新分支进行。
