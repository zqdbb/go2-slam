# 示例栅格地图

本目录随仓库提供 **`my_room.yaml` / `my_room.pgm`**，供 **Nav2 导航** 开箱测试（`go2_nav2.launch.py` 默认加载该地图）。

- 地图为实验室环境建图结果，**坐标系与现场一致时**可直接用于 **2D Pose Estimate + Nav2 Goal**；若在你方场地使用，建议自行建图后替换本目录文件，或启动时用 `map:=/你的路径/xxx.yaml` 指定。
- 保存新地图：建图运行时执行 `ros2 run nav2_map_server map_saver_cli -f <前缀>`，将生成的 `.yaml` 与 `.pgm` 复制到本目录（或任意路径并在 launch 中传入）。
