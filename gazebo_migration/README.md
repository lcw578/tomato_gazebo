# Gazebo 场景

`tomato_corridor_v4.world` 是整机验收场景。它包含两侧共 60 株植株、8 m 轨道及实测的种植支架坡度。v5 沿用 v4 布局，包含重新生成的果梗外形，用于可选查看。实测参数见 [现场数据对照](现场实测数据与v5仿真参数对照.md)。

```bash
cd /data/tomato_gazebo
python3 gazebo_migration/run_classic.py tomato_corridor_v4
python3 gazebo_migration/run_classic.py tomato_corridor_v5 --port 11474
```

带整机查看 v5 时，在 [ROS 2 工作区的启动命令](../ros2_gazebo_ws/README.md#手动启动) 后追加 `world:=/data/tomato_gazebo/gazebo_migration/tomato_corridor_v5.world`。

v4 和 v5 模型包位于 `assets/models/`，两套地图共享 `assets/tomato_v2_common_assets/` 的网格和纹理。v2 世界及真值用作 v4 采摘副本的坐标参考。使用包含 NumPy、SciPy、Trimesh 和 PyYAML 的 Python，依次运行 `build_greenhouse_corridor_v4.py` 和 `build_greenhouse_corridor_v5.py` 可重建测量走廊。v3 脚本提供两个生成器使用的模型工具函数。

`run_classic.py --capture` 使用 Gazebo 世界插件。从本目录构建诊断插件后可采集俯视和果梗图像。

```bash
mkdir -p runtime
c++ -shared -fPIC runtime_probe.cc -o runtime/libmigration_runtime_probe.so \
  $(pkg-config --cflags --libs gazebo)
python3 run_classic.py tomato_corridor_v5 --capture --truss-detail
```
