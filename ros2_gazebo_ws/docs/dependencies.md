# Gazebo 联调依赖和版本管理

## 真机交付基线

本机交付包为 `/home/lcw/Downloads/robot_ws_fixed-v1.0-master.zip`。ZIP 注释记录 Git 版本 `071a3a1ae5932686688d6010642fdaac329ac247`。包和三份变更文件的 SHA-256 见 [交付基线记录](../src/tomato_sim_gazebo/config/robot_source_reference.json)。

[工作区差异补丁](robot_source_workspace_2026-11-20.patch) 记录交付包到本机版本的三处源码差异。主控加入参数和服务就绪处理，YOLO 加入日志与退出清理，真机启动文件停用了腕部反馈节点。交付包里的六条 AUBO SDK 软链接按 ZIP 成员原始内容比较与本机链接目标相同。源码目录中的 Python 字节码属于运行缓存。真机工作区位于项目根目录的 `robot_ws_fixed-v1.0-master/`。Phase 4 直接导入当前真机主控和 YOLO，运行配置使用仿真校准和 Gazebo RGB-D。真机业务入口的依赖由仿真工作区接入。

## 资产来源

| 依赖 | 来源和使用方式 |
| --- | --- |
| v4 温室、轨道、植株模型和纹理 | 新项目的 `gazebo_migration/assets/`。来源为旧 MuJoCo 植株、公共视觉网格和地图生成器。其他环境可用 `TOMATO_GAZEBO_ASSETS` 指定资产目录。生成命令见 [地图说明](../../gazebo_migration/README.md) |
| Phase 3/4 v4 采摘副本 | 从 v4 地图、精细植株 SDF、v2 世界和 `truss_ground_truth.json` 生成 |
| 整机 CAD 网格 | 生成到 `src/tomato_sim_description/meshes/`。已有资产升级到独立轮组时，需一并更新拆分后的车架和 10 个轮组网格。准备 Standalone MJCF 及其引用网格后，用带 NumPy、SciPy 和 Trimesh 的 Python 执行 `scripts/export_mujoco_geometry.py`，再构建 ROS 2 工作区。 |
| AUBO 网格与 URDF | `TOMATO_ROBOT_SOURCE/src/xc_aubo_arm_control/aubo_description/` |
| YOLO 权重 | 从旧项目复制到 `assets/weights/tomato-seg2025063018-yolov8-xucan.pt`，SHA-256 为 `22e7ab93d69bd98cf07411452e3924fc9f126dddcb30fa773d61030eae9b7c92` |
| Python 环境 | 本机 `.venv_ros` 基于系统 Python，`.venv_yolo` 基于 `tomato_sim` Conda 环境。路径覆盖见 [构建说明](../README.md#配置和构建) |

Git 收录联调脚本、ROS 仿真包源码、配置、地图生成器、SDF 世界和小型验收证据。大资产、真实源码链接、覆盖包、环境、构建和运行产物继续保存在本地 `/data`。Phase 1 从 `assets/fk_reference/` 读取复制到新项目的参考模型。

## 验收范围

静止装配基线为 v4、机械臂安装偏航 180°、滑轨初始位 0.30 m。Phase 3/4 的候选采摘工位沿用 v4 基线配置中的数据。场景生成依赖 v2 真值，仲裁器仍使用现有的 v4 采摘释放模型，原始剪切条件继续由既有适配器执行。

2026-11-20 在新项目的默认配置下重新构建了九个包，v4 静止装配通过。v4 采摘副本通过几何审计和 SDF 格式检验。[v4 静止装配报告](evidence/v4_static_2026-11-20.json)

拆分前的 v4 Phase 1 复验已完成按实际世界名查询底盘、三路相机、滑轨和三组 TCP 校验，夹爪闭合位置误差超出原阈值。端到端采摘仍待验收。

文件访问追踪覆盖新项目的构建、v4 启动和静止装配，未出现旧 MuJoCo 仓库路径。v5 启动激活了四个控制器。[拆分复验记录](evidence/standalone_migration_2026-11-20.json)

Phase 4 在 60 秒验证窗口内完成两次仿真 RGB-D 到真机主控及原 YOLO 的请求与响应。每帧最多检出 7 个番茄、5 个主茎和 0 个果梗，未生成剪切点，端到端采摘验收结果为 FAIL。[Phase 4 报告](evidence/v4_phase4_2026-11-20.json)
