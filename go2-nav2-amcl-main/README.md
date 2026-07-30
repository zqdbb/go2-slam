# Unitree Go2 SLAM + 导航（扩展版）

本仓库是在开源项目 **[FishPlusDragon/unitree-go2-slam-toolbox](https://github.com/FishPlusDragon/unitree-go2-slam-toolbox)** 基础上的**二次开发与实机验证**，用于实验室内部学习与复现。原项目侧重 Go2 与 ROS 2 的入门与 **SLAM 建图**；本仓库在尊重原开源协议与作者工作的前提下，补充了**基于 Nav2 的基础路径规划与自主导航**、示例地图、文档与若干工程向修改。

原项目配套视频（入门向）：  
[B 站｜Unitree Go2 相关教程](https://www.bilibili.com/video/BV1caWQzdE3G/?spm_id_from=333.337.search-card.all.click&vd_source=4bd0448ccc277efab1a6915315abd6b9)

更细的环境配置、依赖、调参与排错见：**[docs/功能实现与测试记录.md](docs/功能实现与测试记录.md)**。

---

## 与上游项目的关系


| 说明        | 内容                                                                                                                              |
| --------- | ------------------------------------------------------------------------------------------------------------------------------- |
| **上游仓库**  | [FishPlusDragon/unitree-go2-slam-toolbox](https://github.com/FishPlusDragon/unitree-go2-slam-toolbox)                           |
| **上游作者**  | 感谢 **FishPlusDragon** 等贡献者提供的开源基础（可视化、点云、键盘遥控、EKF、`slam-toolbox` 建图等）。原 README 中关于「学业压力、维护节奏」等说明仍适用于理解上游背景。                     |
| **本仓库扩展** | 集成 **ROS 2 Nav2** 导航栈、示例栅格地图、launch/参数与驱动侧小改，便于在**同一条软件栈**上完成「建图 → 保存地图 → 加载地图导航」。                                              |
| **开发说明**  | 导航相关模块在开发过程中**大量借助 AI 辅助**快速打通流程，**难免存在参数未臻完善、边界情况未覆盖、体验粗糙之处**，适合作为课程/课题组入门与演示；**欢迎后续在深入学习 ROS 2 与移动机器人基础后持续优化**，也欢迎提 Issue 讨论。 |


若你使用或二次分发本仓库，请**保留对上游项目的致谢与链接**；引用本仓库时也可注明为「基于 FishPlusDragon 项目的扩展」。

---

## 实机验证情况（本仓库维护者）

已在 **Ubuntu 22.04 + ROS 2 Humble**、**网线连接 Go2 EDU** 环境下完成：

- **SLAM 建图**与**地图保存**；
- **Nav2 导航**（含 RViz 初始位姿估计与目标点下发）；
- 在 **上海应用技术大学第一学科楼四楼回字型走廊** 完成**建图与实机跑通**（示例地图 `go2_navigation2/maps/` 即来自该场景之一；他人复现时若场地不同，请自行建图或替换地图）。

---

## 硬件与系统

1. Unitree Go2 **EDU**（需具备机载雷达等 ROS 2 话题能力）
2. **Ubuntu 22.04**，安装 **ROS 2 Humble**
3. 网线连接机器狗与 PC（PC 侧常见网段为 `192.168.123.x`，以宇树文档为准）

---

## 本仓库当前能力概览

- 继承上游：**RViz2 模型**、点云累积、`PointCloud2` → `LaserScan`、键盘遥控、`robot_localization`（EKF）、**slam-toolbox** 建图  
- 本仓库新增/强化：**Nav2 路径规划与跟踪**、默认加载包内示例地图、文档与 `.gitignore` 等，便于实验室同学克隆后按文档复现

详细变更与参数说明见 `docs/功能实现与测试记录.md`。

---

## 依赖（必做）

除 ROS 2 Humble 外，需按宇树官方流程安装并编译 **[unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)**（提供 `unitree_go`、`unitree_api`），并配置 **Cyclone DDS** 与接狗网卡（见文档）。

常用 apt 依赖示例（按需补全）：

```bash
sudo apt update && sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  python3-empy
```

编译本工作空间前请先：

```bash
source /opt/ros/humble/setup.bash
source <你的路径>/unitree_ros2/cyclonedds_ws/install/setup.bash
```

再于**工作空间根目录**执行 `colcon build`。若使用 Conda，编译前建议 `**conda deactivate`**，避免错误占用 `python3` 导致消息包编译失败。

---

## 快速启动

```bash
# 1) 宇树 ROS 2 环境（脚本名与网卡请按本机修改）
source ~/unitree_ros2/setup_go2.sh

# 2) 克隆后的工作空间根目录（含 src/）
cd /path/to/your_ws
colcon build
source install/setup.bash
```

**SLAM 建图：**

```bash
ros2 launch go2_core go2_start.launch.py
# 新终端（同样先 source 上述环境）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**保存自建地图（可选）：**

```bash
mkdir -p ~/go2_maps
ros2 run nav2_map_server map_saver_cli -f ~/go2_maps/my_map
```

**Nav2 导航（默认使用包内 `go2_navigation2/maps/my_room.yaml`）：**

```bash
ros2 launch go2_navigation2 go2_nav2.launch.py
# 指定其他地图：
# ros2 launch go2_navigation2 go2_nav2.launch.py map:=/完整路径/xxx.yaml
```

> 建图时建议线速度约 **0.3 m/s**，步态选「经典模式」更稳。导航时在 RViz 中先 **2D Pose Estimate**，再 **Nav2 Goal**。

---

## 仓库内示例地图

路径：`**src/go2_navigation2/maps/`**（`colcon build` 后位于 `install/.../share/go2_navigation2/maps/`）。  
默认导航 launch 会加载该目录下的 `**my_room.yaml` / `my_room.pgm**`。若在你方场地使用，请自行建图替换或通过 `map:=` 指定路径；详见该目录下 `README.md`。

---

## 致谢

- **FishPlusDragon** 及 [unitree-go2-slam-toolbox](https://github.com/FishPlusDragon/unitree-go2-slam-toolbox) 原仓库贡献者：提供清晰易跟的 Go2 + ROS 2 入门基础。  
- **宇树科技** 官方文档与 [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)。  
- **ROS 2 Nav2** 与 **slam_toolbox** 等开源社区。

本扩展仅供学习与交流；实机使用时请注意安全，并在开阔、可控环境中测试。