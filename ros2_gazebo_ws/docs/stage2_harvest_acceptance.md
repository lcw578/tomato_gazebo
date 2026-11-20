# 阶段二采摘闭环验收记录

## 当前验收结果

真值驱动采摘完成两轮局部重置复测，两轮均经过实际刀片接触、严格闭爪、持果运输、动态释放和全果实体筐内静置。原样真机主控已经独立启动并收到真实 GT 识别服务响应，向 `/move_to_pose` 发送请求后取得 `30003` 逆解失败。完整原主控采摘入筐成功周期为 0，3–5 次重复验收仍待末端安装坐标基准核实。[本轮实测汇总](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/stage2_harvest_2026-11-20.json)

## 执行基准

本轮工作基于提交 `933efbd`，分支为 `codex/harvest-closure-stage2`。恢复点为标签 `geometry-actuators-2026-11-20`。机械臂安装采用用户确认的 180° 朝向，底盘沿轨运动采用 `machine_slide`。

当前实测场景为 v4，目标模型为 `fruit_truss_9_1`。该模型代表一串果实，果实体碰撞形状由 12 个球组成。真值进刀采用轨道站位 1.25 m、升降高度 0.85 m，以及刀具切向对齐后的绕自身 X 轴 90° 朝向。以上站位和进刀朝向是本轮选择的仿真验收配置。

`robot_ws_fixed-v1.0-master` 相对基准提交的差异检查为空。视觉诊断记录了以下源文件 SHA-256。

| 源文件 | SHA-256 |
| --- | --- |
| `xc_robot_main_control_node.py` | `ddedeeccce38916ab5af97a65a684524fc8f4738fd99ba13f8d56547d067b60f` |
| `stemGrowthAndPoints.py` | `ebc0b140afcbe7430c70b90bccec452059e65b4f48e44340f1ffd217a6a9e6db` |

## 四项前置检查

### 接触坐标与判据

仲裁器使用 `T_cutter_from_world` 对世界接触点做 `Rp+t`，对果梗切向只做旋转和归一化。切向检查采用 `abs(dot)`，正反两个方向等价。回归测试覆盖刀具原点、旋转和平移分离、反向切向与零力接触。

实际 ODE 刀片接触样本在刀座系下约为 `[-4.8804, -4.1857, -84.9625] mm`，刀片平面偏差 0.0169 mm，方向夹角 0.001°。实际传感器插件上报世界接触坐标，因此刀片消息帧名设为 `world`。[仲裁器](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/harvest_physics_arbiter.py) · [传感器定义](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro:172)

### 接触检测与闭爪

目标果梗使用 `collide_without_contact=true`。携果模型的果实体也采用无排斥力接触，动态释放时恢复果实体的刚体接触。主茎和果筐保留实体碰撞。[场景生成](/data/tomato_gazebo/ros2_gazebo_ws/scripts/prepare_v4_harvest_world.py:154)

真值进刀实测闭爪残差分别为 0.00370 mm 和 0.00388 mm。严格到位门限为双指各小于 0.4 mm。另修复了闭爪完成回调先于接触传感器回调到达的竞态，闭合事件最多等待 0.5 s 仿真时间供接触样本到达。[夹爪到位检查](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_gripper_node.py:17)

异步释放同时覆盖抓取请求期间收到打开事件、关闭取消排队打开、删除成功后重建失败直接重试 Spawn，以及旧 SDF 缺少 `kinematic` 或 `gravity` 标签的情况。

### 腕部服务调用

原主控默认 `is_two_stage=False`，当前单阶段采摘从侧相机计算剪切点。腕部两阶段服务存在 8 项解包与果梗函数实际返回 9 项的差异，列为后续真机缺陷核对项。[主控开关](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:63) · [果梗函数返回](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/stemGrowthAndPoints.py:932)

### 共用仿真基线

`baseline.launch.py` 和 `phase3.launch.py` 均复用 `phase2.launch.py`，继而加载同一机器人描述和控制插件。驻车制动、夹爪内部接触过滤、机械臂装配和实时 TF 回筐 IK 均通过该链路加载。`phase3` 的侧相机深度默认值为 4000 count/m。本轮原主控视觉验证显式使用 1000 count/m，对应其真实相机路径中写定的 `depth*0.001`。[Phase3 启动](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/launch/phase3.launch.py:17) · [共享插件](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_control.xacro:53) · [原主控深度换算](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:498)

## Stage 2A 实测

最终两轮测试取得以下结果，均通过本轮验收脚本。

| 项目 | 第 1 轮 | 第 2 轮 |
| --- | --- | --- |
| TCP 到真值剪切点 | 0.03403 mm | 0.03403 mm |
| X1 与进刀反馈 | 1 与 `result=2` | 1 与 `result=2` |
| 双指闭爪残差 | 约 0.00370、0.00388 mm | 约 0.00370、0.00388 mm |
| 刀片与果梗夹角 | 约 0.001° | 约 0.001° |
| 剪离与主茎接触 | `CUT_SUCCESS`，主茎 0 次 | `CUT_SUCCESS`，主茎 0 次 |
| 回退距离 | 99.5000 mm | 100.0001 mm |
| 停止后持果位置误差 | 0.5003 mm | 0.00000383 mm |
| 最终线速度 | 0.005440 m/s | 0.005531 m/s |
| 低速持续时间 | 0.5 s 仿真时间 | 0.5 s 仿真时间 |
| 全部 12 个果球筐内 | 通过 | 通过 |

第 1 轮在回退服务返回后，第一次采样的持果误差为 99.799 mm，继续采样后在 0.3 s 仿真时间内收敛到 0.5003 mm。第 2 轮第一次采样已经收敛。脚本现在等待运动结束后的新 TF，并按 50 ms 仿真时间间隔核验携果位置，最多等待 5 s。全过程运动中的最大跟随误差尚未记录，这一项保留为运行时同步问题继续核对。[第 1 轮原始数据](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/truth_harvest_cycle1.json) · [第 2 轮原始数据](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/truth_harvest_cycle2.json)

### 升降平台对落筐路径的遮挡

果筐安装在底盘上，世界高度约 0.946379 m。升降高度 0.85 m 时，升降平台上升到约 1.575 m，原回筐策略将 TCP 放在约 1.82038 m。果实下落路径经过升降平台。FCL 扫掠发现原释放朝向在下落约 30 mm 时开始遇到刀片，约 230 mm 时开始遇到平台。

本轮为当前站位配置四个回筐关节点，经过预回零、OEM 零位、侧向绕行和低位释放，TCP 释放位置约 `[0.877144,-0.029137,1.100000] m`。876 个携果离散几何样本中，配置路径的机器人最小间隙为 9.122 mm，全过程主茎最小间隙为 10.217 mm。退刀起始 11 mm 内保留切口接触，起退 20 mm 后额外几何最小间隙为 3.040 mm。空爪通过侧向点返回 OEM 零位，395 个样本的机器人最小间隙为 10.255 mm。采样最大关节间隔为 0.012 rad。[运动配置](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/config/stage2_truth_motion.yaml) · [回筐服务](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_aubo_motion_service.py:283)

该配置针对站位 1.25 m、升降 0.85 m 和真值进刀绕刀轴 90°，其他站位与抓取朝向需要重新核验路径。验收脚本对全部 12 个果球应用最终旋转和位移，检查筐内壁与底板边界，允许 2 mm 数值接触容差。两轮最终果球均在几何边界内，超界量为 0。[全果实体几何验收](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/harvest_physics_arbiter.py:65)

### 同名重建造成释放位置回到植株

低位释放首次复测仍落在地面。独立重建采样证实，请求世界位置为约 `[0.865814,-0.017550,1.120047] m`，实际出生位置却为原始植株位置 `[1.199907,-0.687165,2.064180] m`，且旋转回到单位四元数。[现场采样](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/dynamic_release_probe.json)

Gazebo Classic 的 `Model::Load` 优先从世界 DOM 取得同名模型的语义位姿，因此删除后同名重建会复用原始装配位置。本轮将动态实体命名为 `fruit_truss_9_1_released`，释放证据保存 `model_name`，后续静置验收根据该名称读取实际动态实体。[Gazebo 模型加载源码](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/gazebo/physics/Model.cc#L67-L122) · [上游同名重建问题](https://github.com/gazebosim/gazebo-classic/issues/3238) · [仿真侧修复](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/harvest_physics_arbiter.py:485)

### 果串质心和惯量

原模型的质心位于果梗原点，与 12 个果实体球计算出的质心相差 153.686 mm。独立名称释放后，旧惯性使果串翻成竖立状态，果球高出筐口约 141.8 mm。保留原模型总质量 0.15 kg，按各球体积和均匀密度分配质量后，局部质心为 `[0.0086823923,-0.0041072338,-0.1533856675] m`。惯量由实心球自身惯量和平行轴定理计算，主惯性均为正。该均匀密度分布是本轮采用的仿真假设。[惯量生成](/data/tomato_gazebo/ros2_gazebo_ws/scripts/prepare_v4_harvest_world.py:51) · [回归测试](/data/tomato_gazebo/ros2_gazebo_ws/tests/test_payload_inertia.py)

## Stage 2A 复测方式

使用已加载 v4 场景的 ROS Domain。机械臂运动服务采用下列配置，参数中的 `request_gripper_length=0.0` 对应真值 TCP 驱动。启动前应结束该 Domain 中已有的同名运动服务进程，使服务实例唯一。

```bash
cd /data/tomato_gazebo/ros2_gazebo_ws
source /opt/ros/humble/setup.bash
source .overlay/install/local_setup.bash
export ROS_DOMAIN_ID=138 ROS_LOCALHOST_ONLY=1
ros2 run tomato_sim_adapters sim_aubo_motion_service --ros-args \
  --params-file src/tomato_sim_gazebo/config/stage2_truth_motion.yaml
```

每轮测试前，停止上一轮仲裁器并局部恢复目标。第一轮机械臂已经在 OEM 零位时执行恢复命令。后续机械臂位于本轮低位释放姿态时，增加 `--exit-basket`，沿已检查的空爪路径回零。恢复脚本同时清理原名和 `_released` 动态名。

```bash
python3 scripts/stage2_reset_payload.py --exit-basket
ros2 run tomato_sim_adapters harvest_physics_arbiter --ros-args \
  -p use_sim_time:=true \
  -p scene_file:=$PWD/validation/phase3/phase3_scene.json \
  -p evidence_file:=$PWD/validation/stage2/runtime.json
```

在另一个已加载同一 ROS 环境的终端中执行测试。

```bash
python3 scripts/stage2_harvest_checks.py
```

## Stage 2B 视觉子链路

`sim_gt_yolo_service` 提供 `/yolo_detection`，沿用 `xc_interfaces/srv/YoloDetection`。输入是同曝光的彩色和对齐深度图，响应原样返回两张图。类别、置信度、展平框和 `Mask` 数组逐项同序。掩膜为完整尺寸 `mono8`，前景值为 255，掩膜名称字段为 `class_names`。[服务实现](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_gt_yolo_service.py:81)

几何投影采用同曝光 `CameraInfo` 和 TF，光栅化真实 SDF 视觉几何。目标果梗、果实和主茎之间使用最近表面判定，其余物体遮挡由实测对齐深度裁剪，阈值为 ±12 mm。[投影实现](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/gt_geometry_projection.py)

当前服务加载场景指定的目标植株和目标果串初始几何，适用于局部复原后的采摘前曝光。动态移动后的目标位姿更新属于后续扩展范围。

1000 count/m 深度编码下，三个独立曝光均通过服务契约及原视觉函数检查。

| 项目 | 三帧实测 |
| --- | --- |
| 曝光时间 | 957.806、959.807、961.606 s |
| 原算法骨架点数 | 每帧 22 |
| 原主控匹配对数 | 每帧 1 |
| 剪切点总误差 | 3.9184 mm |
| 沿果梗轴向误差 | +1.8060 mm |
| 法向误差 | 3.4774 mm |
| 原主控停车后深度 | 0.771 m |
| 原主控首次观察掩膜平均深度 | 0.775635 m |
| 距离与深度门限 | 三帧通过 |

此数据来自真实 ROS 服务调用及隔离执行的原视觉函数。原样主控进程测试已取得真实逆解失败响应，完整周期成功次数为 0。[三帧汇总](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/vision/vision_subchain_summary.json)

### 原主控姿态与剪刀坐标核对

同一组图像经原函数处理后产生 `mid2target_angle=0`，RPY 为 `[90°,70°,180°]`，机械臂基座系请求坐标为 `[0.0486071,0.6072222,0.4552814] m`，包含 0.10 m 夹具长度补偿。当前桥接恢复的 TCP 目标为 `[0.0486071,0.7072222,0.4552814] m`。

按当前桥接的 TCP 朝向解释，刀具切向与果梗夹角为 85.6449°。按 SDK 法兰朝向并组合现有刀座安装旋转计算，夹角为 88.4162°。剪切判据要求夹角小于等于 30°。实际启动原样主控后，日志记录了同一组请求，运动服务在中间点逆解阶段返回 `30003`。中间点所需 wrist3 位置为机械臂底座系 `[0.04279655,0.74317419,0.45218180] m`。多初始种子离线逆解也未得到解。末端安装旋转及 SDK TCP 配置仍需真机依据。[原样主控日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/vision/native_main.log:88) · [运动服务日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/motion_service_native.log:1) · [主控固定俯仰](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:1416) · [硬件姿态转换](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_aubo_arm_control/src/motion_service.cpp:1069) · [SDK 法兰接口说明](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_aubo_arm_control/include/aubo_sdk/serviceinterface.h:605)

![剪刀与果梗朝向](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/visuals/cutter_direction_comparison.png)

### 需要真机提供的直观参考

先提供一张同时看到机械臂手腕、剪刀固定座和刀片的侧面照片，便于核对现有 CAD 的装配关系。若有 AUBO 工具配置页面，再提供该页面中工具位置和旋转值。当前 CAD 的法兰至刀座旋转为 `[180°,0°,90°]`，而旧真机 URDF 使用简化夹具模型，无法据此恢复刀片的实际方向。[CAD 安装旋转](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/mujoco_cad.xacro:540)

原样主控在收到逆解失败后已停止。其 Python 进程在 SIGINT 和 SIGTERM 后仍保留线程，本轮终止了该测试进程，并将 AGV 连续运行参数设为 false。

## 复现视觉诊断

在运行中的同一 ROS Domain 内启动 1000 count/m 的相机桥接和 GT 服务，然后执行以下诊断命令。

```bash
source /opt/ros/humble/setup.bash
source .overlay/install/local_setup.bash
ROS_DOMAIN_ID=138 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  .venv_ros/bin/python scripts/stage2_gt_vision_checks.py \
  --output validation/stage2/vision/recheck \
  --depth-units-per-metre 1000 --gripper-length .10 \
  --ros-args -p use_sim_time:=true
```

## 驻车与时间基准更正

独立物理位姿复测覆盖三段机械臂轨迹，Gazebo 车架世界 X 峰值漂移约 `3.96e-8 mm`，关节采样峰值约 `9.54e-9 mm`。有效测时中仿真推进 1.800 s、墙钟耗时 1.800717 s，对应约 0.9996 倍速。此前报告的 80.47 倍速采用未初始化的时钟起点，已更正。[预检证据](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/stage2_preflight_2026-11-20.json)

## 最终代码检查

本轮完整单元测试 44/44 通过，包含阶段一既有 10 项回归。Overlay 构建的 9 个包全部通过，v4 场景完整性检查通过，保留 94 个原始实体和 60 株植株。真机业务目录对基准提交的差异检查为空。

Gazebo 固定关节合并后，果筐碰撞名包含 `crate_col_*`。接触分类现同时识别该名称和 `basket_link`，回归测试使用现场碰撞名。两轮物理测试的历史接触日志保留原始分类，全果筐内结论来自独立实体位姿与 12 个球的几何检查。
