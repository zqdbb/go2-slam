# 项目备份与完整复现

本文档说明如何备份、恢复和复现 Go2 MuJoCo + RL 步态 + SLAM Toolbox + Nav2 + SCAN-Planner 集成项目。

## 1. 备份内容

项目由三部分组成：

| 内容 | 备份位置 | 说明 |
|---|---|---|
| 源码、launch、配置、RViz、场景 | GitHub 代码仓库 | 可版本管理、可回退 |
| 地图、RL policy、MuJoCo 网格 | GitHub 项目目录 | 当前基线已包含 |
| ROS 2 / Python / MuJoCo 系统依赖 | GHCR Docker 镜像 | 镜像上传需要 \`write:packages\` |

代码仓库：

    https://github.com/zqdbb/go2-slam

项目目录：

    go2_mujoco_slam_scan_navigation/

当前代码基线：

    commit: ccb895d9ad319ea2b64662d0687bdb82e07f88bf
    tag:    go2-mujoco-slam-scan-baseline-20260821

## 2. 当前代码备份

本机正式修改目录：

    ~/桌面/go2slam/go2_mujoco_slam_scan_navigation

检查工作区并提交：

    cd ~/桌面/go2slam
    git status --short --branch
    git add go2_mujoco_slam_scan_navigation
    git commit -m "Describe the backup or code change"
    git push origin main

创建新的可回退标签：

    git tag -a go2-baseline-$(date +%Y%m%d) \
      -m "Go2 navigation backup"
    git push origin --tags

## 3. 从服务器同步源码

服务器项目：

    hongzt@192.168.100.55:~/Desktop/go2/SCAN-Planner-Ros2

同步到本机项目目录时不要复制远程 \`.git\`、构建产物或残损 Git 目录：

    rsync -a --info=progress2 \
      --exclude='/.git/' \
      --exclude='/.git.incomplete-*/' \
      --exclude='/core' \
      hongzt@192.168.100.55:~/Desktop/go2/SCAN-Planner-Ros2/ \
      ~/桌面/go2slam/go2_mujoco_slam_scan_navigation/

同步后检查关键文件：

    test -f ~/桌面/go2slam/go2_mujoco_slam_scan_navigation/mujoco_server.py
    test -f ~/桌面/go2slam/go2_mujoco_slam_scan_navigation/policies/model_40000.pt
    test -f ~/桌面/go2slam/go2_mujoco_slam_scan_navigation/maps/go2_mapping_20260820_105700.posegraph
    test -f ~/桌面/go2slam/go2_mujoco_slam_scan_navigation/maps/go2_mapping_20260820_105700.data

## 4. 地图备份

SLAM Toolbox 地图必须成套保存：

    maps/go2_mapping_20260820_105700.posegraph
    maps/go2_mapping_20260820_105700.data
    maps/go2_mapping_20260820_105700.pgm
    maps/go2_mapping_20260820_105700.yaml

\`.posegraph\` 和 \`.data\` 是定位/位姿图恢复所需文件；只有 \`.pgm\` 不能恢复完整 SLAM 状态。

保存新地图后：

    mkdir -p maps/go2_mapping_YYYYMMDD_HHMMSS
    cp new_map.posegraph new_map.data new_map.pgm new_map.yaml \
      maps/go2_mapping_YYYYMMDD_HHMMSS/
    git add maps
    git commit -m "Add SLAM map YYYYMMDD_HHMMSS"
    git push origin main

## 5. Docker 镜像备份

当前运行环境对应两个镜像：

    go2_humble:latest                 约 6.42 GB
    mujoco-huanghb:pi05-libero        约 18.3 GB

GHCR 目标标签：

    ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821
    ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821

登录 GHCR 需要 GitHub Personal Access Token：

    read:packages       拉取镜像
    write:packages      上传镜像

服务器登录：

    echo "$GITHUB_TOKEN" | docker login ghcr.io \
      --username "$GITHUB_USER" --password-stdin

打标签并上传：

    docker tag go2_humble:latest \
      ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821
    docker tag mujoco-huanghb:pi05-libero \
      ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821

    docker push ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821
    docker push ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821

当前已知状态：代码和本文档已上传 GitHub；此前服务器 Token 缺少 \`write:packages\`，两个镜像尚未成功上传。上传成功后，验证：

    docker manifest inspect ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821
    docker manifest inspect ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821

## 6. 无 GHCR 时的离线镜像备份

如果暂时没有 \`write:packages\`，可以先在服务器导出镜像到外部磁盘。不要把这些压缩包提交到普通 GitHub 代码仓库：

    docker save go2_humble:latest | gzip > go2_humble_integrated_20260821.tar.gz
    docker save mujoco-huanghb:pi05-libero | gzip > mujoco_huanghb_pi05_libero_20260821.tar.gz
    sha256sum *_20260821.tar.gz > docker-images.sha256

恢复：

    sha256sum -c docker-images.sha256
    gunzip -c go2_humble_integrated_20260821.tar.gz | docker load
    gunzip -c mujoco_huanghb_pi05_libero_20260821.tar.gz | docker load

## 7. 从 GitHub 恢复代码

    mkdir -p ~/桌面
    git clone https://github.com/zqdbb/go2-slam.git ~/桌面/go2slam
    cd ~/桌面/go2slam
    git checkout go2-mujoco-slam-scan-baseline-20260821

恢复后的源码目录：

    ~/桌面/go2slam/go2_mujoco_slam_scan_navigation

## 8. 恢复服务器目录

    ssh hongzt@192.168.100.55 'mkdir -p ~/Desktop/go2/SCAN-Planner-Ros2'
    rsync -a --delete \
      --exclude='/.git/' \
      --exclude='/build/' \
      --exclude='/install/' \
      --exclude='/log/' \
      ~/桌面/go2slam/go2_mujoco_slam_scan_navigation/ \
      hongzt@192.168.100.55:~/Desktop/go2/SCAN-Planner-Ros2/

\`--delete\` 只允许用于明确的项目目录，执行前必须确认目标路径正确。

## 9. 恢复容器

ROS 2 容器至少需要：

    docker run -d --name go2_mapping \
      --gpus all --network host --shm-size=64m \
      -v "$HOME/Desktop/go2/SCAN-Planner-Ros2:/workspace" \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v "$HOME/.Xauthority:/tmp/.Xauthority" \
      -v /var/run/docker.sock:/var/run/docker.sock \
      ghcr.io/zqdbb/go2-slam/go2-humble:integrated-20260821 \
      bash /workspace/start_mapping.sh

MuJoCo 容器：

    docker run -d --name mujoco-go2-mapping \
      --shm-size=64m \
      -v /home/huanghb/workspace/lerobot:/workspace/lerobot \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v /home/huanghb/.Xauthority:/tmp/.Xauthority-host \
      ghcr.io/zqdbb/go2-slam/mujoco-huanghb:pi05-libero-20260821 \
      tail -f /dev/null

## 10. 重要的容器名说明

当前 \`integrated_navigation.launch.py\` 中配置的 MuJoCo 容器名是：

    mujoco-go2-mapping

服务器历史上曾使用过：

    mujoco-huanghb-pi05-teleop

两者必须统一。否则需要修改 launch 文件中的 \`mujoco_container\` 参数，或将容器命名为 \`mujoco-go2-mapping\`。

## 11. 不应备份到 GitHub 的内容

以下内容是运行产物或敏感/超大文件，应由 \`.gitignore\` 排除：

    build/
    install/
    log/
    __pycache__/
    core
    libmujoco.so*
    临时 map.pcd
    None.posegraph
    None.data

代码、配置、地图、policy 和 MuJoCo 场景文件应保留在 GitHub 项目目录中。
