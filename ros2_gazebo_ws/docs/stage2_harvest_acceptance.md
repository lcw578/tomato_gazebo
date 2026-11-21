# 阶段二采摘闭环验收记录

## 2026-11-21 零升降单株复测

单株场景将升降平台保持在 0 m，果筐固定在升降平台，目标果串相对旧 v4 位置下移 0.85 m。回筐适配器读取当前果筐 TF，并以当前关节状态求解释放位姿，不再执行旧配置中的四点固定关节绕行。原样主控调用 `/return_to_box` 后约 5.8 s 返回果筐并成功开爪。末端相对 `basket_link` 的释放位姿为 `[0, 0, -0.220] m`。

单次端到端复测 `1/1` 通过。流程到达 `ATTACHED_TO_STEM → HELD_BY_GRIPPER → RELEASED`。验收只要求 12 个果实碰撞球的包络进入果筐内腔，保留 2 mm 数值接触容差，不要求载荷停止运动、静置、速度或姿态稳定。该轮结果为 `inside=true`，最大边界超出量 1.589 mm，落在 2 mm 容差内。剪切点 TCP 误差 3.9701 mm，刀面与果梗交角 1.4098°，夹爪闭合残差最大 0.000231 mm，主茎接触 0 次，滑轨漂移 0.0000203 mm。原主控与 `stemGrowthAndPoints.py` 运行前后哈希一致，完整测试集 71/71 通过。[单轮验收证据](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/single_plant_lift_zero/basket_only_acceptance/native_harvest_summary.json) · [回筐路径实现](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_aubo_motion_service.py:337) · [当前运动配置](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/config/stage2_native_motion.yaml:1)

## 2026-11-20 稳定静置验收结果（历史记录）

2026-11-20，最终代码在受控单株场景完成连续 3/3 轮原样主控物理动作闭环。每轮均经过原视觉函数、实际进刀接触、严格闭爪、携果回筐、原主控成功开爪和动态落筐。原主控及 `stemGrowthAndPoints.py` 运行前后 SHA-256 核验一致。完整单元测试 64/64 通过，overlay 九个包构建通过。[已提交的测量证据](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/stage2_native_harvest_2026-11-20.json) · [本地完整原始数据](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/native_acceptance/native_harvest_summary.json)

| 实测项目 | 第 1 轮 | 第 2 轮 | 第 3 轮 |
| --- | --- | --- | --- |
| 原算法两次骨架点数 | 168 / 168 | 168 / 168 | 168 / 168 |
| 剪切点 TCP 误差 | 3.185098 mm | 3.185103 mm | 3.185107 mm |
| 刀面法向与果梗交角 | 0.248116° | 0.248116° | 0.248116° |
| 左 / 右指闭爪残差 | 0.000190 / 0.000011 mm | 0.000190 / 0.000011 mm | 0.000190 / 0.000011 mm |
| 主茎接触次数 | 0 | 0 | 0 |
| 启动至落筐静置全过程底盘漂移 | 0.00004764 mm | 0.00004786 mm | 0.00004754 mm |
| 静置窗口最大位置跨度 | 0.429845 mm | 0.408321 mm | 0.451829 mm |
| 静置窗口最大姿态跨度 | 0.188671° | 0.183277° | 0.197119° |
| 持续筐内检查的样本数 | 521 | 513 | 545 |
| 果球筐内包络 | 12 / 12，超界 0 | 12 / 12，超界 0 | 12 / 12，超界 0 |

以上结果对应完整 TCP 平移适配、136 mm 运行时长度、单株合成果梗和 GT 识别。独立单果资产、密植场景及真实 YOLO 的验证范围见下文。原主控内部采摘成功标志存在已确认的注释赋值问题，业务状态处理需要单独解决。[业务状态诊断](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:1975)

## 完整 TCP 补偿与单株验收配置

以下工具参考、场景参数和稳定静置测量记录对应 2026-11-20 的 0.85 m 升降配置。当前零升降配置与入筐判据见本页的 2026-11-21 复测结果。

用户已授权仿真侧完整 TCP 补偿及单株验收场景。原主控通过运行时参数加载 136 mm 工具长度，运动服务采用 `sdk_flange`、1000 count/m 深度和 `sim_gt_yolo_service`。本轮配置基于提交 `83f69b7`。[主控运行配置](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/config/stage2_native_main.yaml:5) · [运动配置](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/config/stage2_native_motion.yaml:5)

### 工具参考点适配

当前 CAD 的 wrist3 至刀口向量为 `[-4.4,-4.9,135.952] mm`。136 mm 标量补偿与它的轴向差为 0.048 mm，横向差为 6.5856 mm。启用 `compensate_native_tool_offset` 后，仿真对 SDK 请求的中间点与终点同时增加 `R × ([0,0,0.136] − p_wrist_to_tcp)`，再求解法兰运动。因此仿真实际执行的法兰平移包含显式校正，`/arm_status` 回报实际法兰位姿。[补偿实现](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_aubo_motion_service.py:250)

横向偏置可追溯到旧装配工具里的 `TCP_BASE=[-4.9,-4.4,-85] mm`，经固定座旋转得到当前腕部向量。完整刀片网格的刀面 X 约为 −4.897326 mm，与参考 X 相差 0.002674 mm。TCP 的 Y=−4.4 mm 位于刀槽范围 `[−7.5,2.5] mm` 内，距刀槽几何中点 −2.5 mm 为 1.9 mm。该点承接历史接触 TCP 定义，真实工具的绝对偏心仍需实物测量确认。[参考装配](/data/robot_assembly/tools/scene_assets.py:15) · [刀口定义](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro:115)

该开关在节点启动时读取。启动参数按内置缺省值、`motion_config`、显式 launch 参数依次覆盖，因此加载本轮配置后，`compensate_native_tool_offset:=false` 可关闭补偿。原有调用入口的缺省值为关闭。[参数加载](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/launch/phase2.launch.py:46)

### 单株场景与运动路径

验收场景保留一个目标植株及其实体主茎，配置一个可剪离果串。果梗采用直径 5 mm、长度 240 mm 的直圆柱，两段各 120 mm，剪切真值取几何中点。果梗世界方位角为 −32°、仰角为 −20°，沿 +X 侧伸并向下倾斜。该几何为本轮选择的受控验收样例。[生成器](/data/tomato_gazebo/ros2_gazebo_ws/scripts/prepare_side_peduncle_world.py:50)

目标载荷沿用原果串的视觉网格、12 个果球碰撞体及 0.15 kg 总质量。释放后恢复刚体接触与重力。质量分配采用既有均匀密度假设。本轮验证范围为单株、单个果串载荷和 GT 识别，独立单果资产、完整 v4 密植环境、真实 YOLO 与真机工具绝对几何仍待各自验收。

进刀前增加仿真关节绕行点，以避免 OEM 初始位姿直达中间点时碰到主茎。携果回筐经过绕行、OEM 零位、侧向点和低位释放，释放时将果串长轴对齐果筐长边。离线 FCL 检查覆盖 1374 个关节离散样本，最大采样步长 0.012 rad，最小间隙 10.2552 mm，下落路径扫描的机器人干涉数为 0。该结果对应本轮站位 1.25 m、升降 0.85 m、相机偏航 32° 和指定路点。[运动配置](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/config/stage2_native_motion.yaml:9) · [离线扫掠证据](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/single_plant_oblique/verified_route_long_y.json)

### 原样主控验收流程

每轮在局部复原后启动原主控进程，由原主控完成识别匹配、骨架提取、两段进刀、闭爪、退刀、回筐和开爪。仲裁器记录实际刀片与指定果梗接触、`CUT_SUCCESS`、`HELD_BY_GRIPPER` 和动态 `RELEASED`。2026-11-20 的历史验收还等待动态释放后的物理静置；当前脚本在释放后轮询果实碰撞体包络，首次确认进入果筐内腔即通过，不等待载荷静止。漂移验收覆盖主控动作至确认入筐。[当前验收脚本](/data/tomato_gazebo/ros2_gazebo_ws/scripts/stage2_native_harvest_checks.py:161)

2026-11-20 历史静置测量采用至少 0.5 s 仿真时间窗口、1 mm 位置跨度、0.5° 姿态跨度、0.02 m/s 线速度和 0.5 rad/s 角速度门限。这些稳定性门限不再用于当前阶段二入筐验收。2 mm 果球包络数值容差仍保留。[历史静置检查](/data/tomato_gazebo/ros2_gazebo_ws/scripts/phase3_checks.py:278)

ODE 接触抖动的历史 10 s、50 样本专项采样中，位置范围为 `[1.541,0.622,0.0467] mm`，相对起点最大姿态变化 0.204°，瞬时角速度最大 0.273 rad/s、中位数 0.102 rad/s。只用 0.1 rad/s 瞬时角速度门限曾在 120 s 后超时。这些数据记录了旧静置判据的调试过程，不属于当前入筐门限。[抖动原始数据](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/jitter_probe.json)

此前尝试发现过主茎路径干涉、果串搭在筐沿、开爪事件异步观察竞态和删除重建竞态，分别通过仿真路点、释放朝向、等待开爪观察及等待 Gazebo 模型列表确认删除处理。旧版仅线速度和新版稳定静置结果均作为历史记录保留；当前阶段二验收仅以果实碰撞体进入果筐内腔作为落筐条件。[局部复原](/data/tomato_gazebo/ros2_gazebo_ws/scripts/stage2_reset_payload.py:38) · [回归用例](/data/tomato_gazebo/ros2_gazebo_ws/tests/test_native_acceptance.py)

原主控开爪成功后会恢复巡航。一次复测中，等待仲裁器 `RELEASED` 后才结束主控，造成落筐期间果筐世界 X 移动约 22.3 mm。当前脚本在原主控报告开爪成功后结束本轮进程，并对从主控启动到确认入筐的轨道漂移设限为 0.5 mm。

### 原主控业务成功标志

启用末端执行器时，`gripper_callback_open()` 中的 `self.is_picking_success=True` 被注释。成功开爪会清零反馈与尝试计数，并结束本串等待，成功标志维持初始化的 False。后续主循环据此将已处理配对加入失败列表。该现有业务状态缺陷由源代码与复测日志共同确认，本轮物理验收分别记录剪切、回筐和开爪成功证据。此项需要在真机业务代码维护范围内另行解决。[开爪回调](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:1965) · [失败列表处理](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:1521) · [复测日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/native_verified/run_02/native_main.log)

### 完整闭环复现

先构建 overlay，再生成受控场景并启动 Gazebo。以下命令的工作目录为 `/data/tomato_gazebo/ros2_gazebo_ws`，各终端加载相同环境。

```bash
source /opt/ros/humble/setup.bash
./build_overlay.sh
source .overlay/install/local_setup.bash
export ROS_DOMAIN_ID=138 ROS_LOCALHOST_ONLY=1
export GAZEBO_IP=127.0.0.1 GAZEBO_MASTER_URI=http://127.0.0.1:11738
python3 scripts/prepare_side_peduncle_world.py \
  --output validation/stage2/single_plant_oblique \
  --direction positive-x --azimuth-offset-deg -32 --single-plant
ros2 launch tomato_sim_gazebo phase2.launch.py \
  world:=$PWD/validation/stage2/single_plant_oblique/tomato_corridor_v4_phase3.world \
  slide_initial:=1.25 lift_height:=0.85 \
  camera_angle_deg:=32 camera_tilt_deg:=0 camera_hfov_deg:=70 \
  gui:=false self_collide:=true left_camera:=false \
  right_camera:=true hand_camera:=false camera_rate:=5 \
  active_camera_side:=right remote_depth_units_per_metre:=1000 \
  motion_config:=$PWD/src/tomato_sim_gazebo/config/stage2_native_motion.yaml
```

第二个终端生成与本次装配对应的 SDK 基座相机外参，启动 GT 识别服务。

```bash
source /opt/ros/humble/setup.bash
source .overlay/install/local_setup.bash
export ROS_DOMAIN_ID=138 ROS_LOCALHOST_ONLY=1
python3 scripts/prepare_phase4_calibration.py \
  validation/stage2/calibration_sdk --pose-reference sdk_flange \
  --ros-args -p use_sim_time:=true
python3 -m tomato_sim_adapters.sim_gt_yolo_service --ros-args \
  -p use_sim_time:=true -p depth_units_per_metre:=1000 \
  -p scene_file:=$PWD/validation/stage2/single_plant_oblique/phase3_scene.json \
  -p evidence_file:=$PWD/validation/stage2/single_plant_oblique/gt_recheck.json
```

第三个终端执行连续三轮。脚本读取指定场景与运动配置，核对运行参数，记录原主控源文件哈希。输出目录保存每轮原主控日志、仲裁器日志和 JSON 证据。

```bash
source /opt/ros/humble/setup.bash
source .overlay/install/local_setup.bash
export ROS_DOMAIN_ID=138 ROS_LOCALHOST_ONLY=1
python3 scripts/stage2_native_harvest_checks.py \
  --scene-file validation/stage2/single_plant_oblique/phase3_scene.json \
  --motion-config src/tomato_sim_gazebo/config/stage2_native_motion.yaml \
  --calibration-dir validation/stage2/calibration_sdk \
  --output validation/stage2/native_recheck --runs 3
```

上一轮已停在本配置释放位姿时，加 `--exit-basket` 可先按已核验的空爪路径返回初始位姿，再局部复原目标。该路径的起点会与实测六关节角比较，容差为 0.08 rad。[复原入口](/data/tomato_gazebo/ros2_gazebo_ws/scripts/stage2_reset_payload.py:18)

## 完整 TCP 适配前的验收记录

真值驱动采摘完成两轮局部重置复测，两轮均经过实际刀片接触、严格闭爪、持果运输、动态释放和全果实体筐内静置。原样真机主控首次运行收到 GT 识别响应后，在 `/move_to_pose` 取得 `30003` 逆解失败。后续核对发现仿真桥接的 TCP 解释与硬件 SDK 法兰接口有差异，已增加 SDK 对齐模式并同步相机外参。该模式下原请求的中间点和目标点离线逆解通过，在线法兰位置反馈与关节 FK 一致。[首轮实测汇总](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/stage2_harvest_2026-11-20.json) · [SDK 对齐核验](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/stage2_wrist_reference_2026-11-20.json)

提交 `83f69b7` 时，SDK 对齐后尚未执行新的原主控运动周期。当时模型复算的刀口距剪切点为 34.3574 mm，刀片法向与果梗轴线夹角为 88.4162°，超出 5 mm 与 30° 的剪切条件。当时完整原主控采摘入筐成功周期为 0。

## 历史执行基准

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

首次测试的图像经原函数处理后产生 `mid2target_angle=0`，RPY 为 `[90°,70°,180°]`，URDF 基座系请求坐标为 `[0.0486071,0.6072222,0.4552814] m`，包含 0.10 m 夹具长度补偿。当时桥接恢复的 TCP 目标为 `[0.0486071,0.7072222,0.4552814] m`。

按该 TCP 朝向解释，刀片法向与果梗轴线夹角为 85.6449°。实际启动原样主控后，日志记录了同一组请求，运动服务在中间点逆解阶段返回 `30003`。当时中间点所需 wrist3 位置为机械臂底座系 `[0.04279655,0.74317419,0.45218180] m`，多初始种子离线逆解也未得到解。这是提交 `214c268` 时的历史失败记录。[原样主控日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/vision/native_main.log:88) · [运动服务日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/motion_service_native.log:1)

硬件 `create_waypoint` 将请求 XYZ 直接作为 SDK 法兰位置，并将 RPY 及归一化后的进刀角直接传给 SDK 逆解。仿真现提供 `pose_reference=sdk_flange` 复现该接口，包括两侧中间点和特殊角度分支。该模式下 `/arm_status` 也使用 SDK 法兰坐标。[硬件路径](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_aubo_arm_control/src/motion_service.cpp:1022) · [SDK 法兰接口说明](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_aubo_arm_control/include/aubo_sdk/serviceinterface.h:605) · [仿真桥接](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_aubo_motion_service.py:60)

官方 SDK 动态库与 URDF 的 102 组关节 FK 对照显示，URDF wrist3 位置恒比 SDK 法兰位置高 23.5 mm，旋转矩阵最大元素误差为 `8.33e-16`。仿真在运动和相机标定两端均按基座 Z 轴处理该偏移。运行中的法兰位置反馈与同时采样关节 FK 的位置差为 `2.48e-12 m`。该在线检查采用静止机械臂。[运动学转换](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/arm_kinematics.py:56) · [标定转换](/data/tomato_gazebo/ros2_gazebo_ws/scripts/prepare_phase4_calibration.py:18) · [原始 SDK 对照](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/sdk_wrist_frame_verification.json)

SDK 基座下的新曝光经过原视觉函数得到请求 `[0.0486071,0.6072222,0.4317814] m`，RPY 为 `[90°,70°,180°]`。中间点 `[0.0486071,0.5072222,0.4317814] m` 和目标点的逆解均通过。真实服务回调的单元测试覆盖完整中间点及目标点求解，执行动作由测试替身记录。复算目标刀口世界位置为 `[1.1972751,-0.7471741,2.0781191] m`，剪切点为 `[1.1999073,-0.7130195,2.0754808] m`，位置差 34.3574 mm，刀片法向与果梗轴线夹角 88.4162°。这些端点数值来自离线 FK/IK。[新曝光诊断](/data/tomato_gazebo/ros2_gazebo_ws/validation/stage2/vision/sdk_reference/vision_diagnostic.json) · [接口回归测试](/data/tomato_gazebo/ros2_gazebo_ws/tests/test_sdk_pose.py)

### 用户确认的初始安装关系

用户确认“腕部相机在正上方、刀口水平”对应机械臂初始位姿，其他姿态允许随手腕旋转。本轮将“水平”按两片刀片所在平面平行地面核验，并以当前启动关节角 `[10.32,15.37,134.05,117.05,86.63,0]°` 为初始位姿。现有 CAD 在该位姿的刀面倾斜 1.6300°，相机中心高于刀口 TCP 114.0436 mm，基本符合这项描述。采摘请求对应姿态的刀面倾斜约 70°。

现有 CAD 的 wrist3 至刀口偏移为 `[-4.4,-4.9,135.952] mm`，主控按 100 mm 夹具长度补偿。轴向名义差为 35.952 mm，实际剪切点误差还叠加视觉定位和横向安装偏移。需要核对真机手腕参考面至刀口的位置及工具设置，才能确定后续仿真修正量。用户给出的初始安装关系支持当前总体装配方向，精确刀口偏移和采摘姿态下的刀片交角仍待核验。[CAD 安装定义](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/mujoco_cad.xacro:540) · [本轮几何数据](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/stage2_wrist_reference_2026-11-20.json)

原样主控在收到逆解失败后已停止。其 Python 进程在 SIGINT 和 SIGTERM 后仍保留线程，本轮终止了该测试进程，并将 AGV 连续运行参数设为 false。

## 复现视觉诊断

原主控对接使用 SDK 模式，启动 `phase3.launch.py` 时传入 `pose_reference:=sdk_flange`，单独启动运动服务时传入 `-p pose_reference:=sdk_flange`。默认 `pose_reference=tcp` 服务于原有真值 TCP 驱动配置。相机标定及视觉诊断应选用相同坐标基准。以下命令在运行中的同一 ROS Domain 内执行，采用 1000 count/m 相机桥接和 GT 服务。

```bash
source /opt/ros/humble/setup.bash
source .overlay/install/local_setup.bash
export ROS_DOMAIN_ID=138 ROS_LOCALHOST_ONLY=1
python3 scripts/prepare_phase4_calibration.py \
  validation/stage2/calibration_sdk --pose-reference sdk_flange \
  --ros-args -p use_sim_time:=true
```

原主控读取的左右相机标定文件应来自上面的输出目录。视觉诊断按同一基准执行。

```bash
ROS_DOMAIN_ID=138 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1 \
  .venv_ros/bin/python scripts/stage2_gt_vision_checks.py \
  --output validation/stage2/vision/recheck \
  --depth-units-per-metre 1000 --gripper-length .10 \
  --pose-reference sdk_flange \
  --ros-args -p use_sim_time:=true
```

## 驻车与时间基准更正

独立物理位姿复测覆盖三段机械臂轨迹，Gazebo 车架世界 X 峰值漂移约 `3.96e-8 mm`，关节采样峰值约 `9.54e-9 mm`。有效测时中仿真推进 1.800 s、墙钟耗时 1.800717 s，对应约 0.9996 倍速。此前报告的 80.47 倍速采用未初始化的时钟起点，已更正。[预检证据](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/stage2_preflight_2026-11-20.json)

## 最终代码检查

SDK 对齐后的完整单元测试 53/53 通过，包含阶段一既有 10 项回归。Overlay 构建的 9 个包全部通过。首次阶段二验收时，v4 场景完整性检查通过，保留 94 个原始实体和 60 株植株。真机业务目录对基准提交的差异检查为空。

Gazebo 固定关节合并后，果筐碰撞名包含 `crate_col_*`。接触分类现同时识别该名称和 `basket_link`，回归测试使用现场碰撞名。两轮物理测试的历史接触日志保留原始分类，全果筐内结论来自独立实体位姿与 12 个球的几何检查。

## 坐标映射实测与可视化追查

用户确认初始外观符合真机安装关系，并要求用可视化和实际运动排查坐标映射。本轮以当前启动关节角 `[10.32,15.37,134.05,117.05,86.63,0]°` 为初始位姿，实际执行 8 个关节姿态和 8 个 SDK 笛卡尔目标。试验结束后通过 `/move_joints` 回到该初始位姿。采摘目标姿态采用原视觉曝光与原主控请求离线复算，本轮新增完整采摘周期为 0。[实测汇总](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/frame_mapping_2026-11-20.json)

### 独立验证 SDK、TF 与 Gazebo 物理位姿

首先在初始位姿附近，依次令六个关节各增加 0.10 rad，最后回初始位姿，共 8 个测点。每次通过正常关节服务运动并静置采样。SDK FK 直接调用已安装官方动态库，实际物理手腕位姿取自 Gazebo `/get_entity_state` 的 `tomato_picker::wrist3_Link`。以这两组独立数据计算世界系至 SDK 基座的刚体变换，再与 TF 比较。

然后使用这 8 个关节测点拟合的刚体变换，预测另一组 SDK 笛卡尔目标。通过正常 `/move_to_pose` 服务验证初始位置、SDK 基座 X/Y/Z 分别增加 10 mm、工具局部 X/Y/Z 分别增加 5°，最后返回初始目标，共 8 个测点。测试采用 `start2mid=true`，按硬件中间点规则反算请求值，使实际中间点恰好落在待测位姿。[SDK 请求构造](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_aubo_motion_service.py:60) · [关节实测脚本](/data/tomato_gazebo/ros2_gazebo_ws/validation/frame_mapping/probe_mapping.py) · [笛卡尔实测脚本](/data/tomato_gazebo/ros2_gazebo_ws/validation/frame_mapping/probe_cartesian.py)

| 检查项目 | 实测结果 |
| --- | --- |
| 8 个关节姿态的物理 wrist3 与 TF 最大位置差 | 0.10769 mm |
| 8 个关节姿态的物理 wrist3 与 TF 最大旋转差 | 0.005641° |
| 8 个笛卡尔服务返回结果 | 全部为 0 |
| 独立笛卡尔目标的最大物理位置误差 | 0.043815 mm |
| 独立笛卡尔目标的最大物理旋转误差 | 0.007892° |
| SDK +X 10 mm 的实际世界位移 XYZ | `[-10.00998,-0.00295,-0.00237] mm` |
| SDK +Y 10 mm 的实际世界位移 XYZ | `[0.00087,-10.00121,-0.00002] mm` |
| SDK +Z 10 mm 的实际世界位移 XYZ | `[-0.00040,-0.00024,9.99981] mm` |

这组测量确定了当前仿真的轴向与符号，SDK 基座约以世界 Z 轴旋转 180°，SDK +X/+Y 对应世界 −X/−Y，SDK +Z 对应世界 +Z。由 8 个关节姿态拟合出的 SDK 基座世界位置为 `[1.25194949,-0.00403172,1.64397113] m`。数据支持 SDK 运动接口、关节执行和 Gazebo 手腕几何之间的映射精度。相机曝光对应的视觉剪切点至场景真值误差为 3.9184 mm。[物理关节原始数据](/data/tomato_gazebo/ros2_gazebo_ws/validation/frame_mapping/live_mapping.json) · [独立笛卡尔原始数据](/data/tomato_gazebo/ros2_gazebo_ws/validation/frame_mapping/live_cartesian.json)

### 34.36 mm 的来源可由工具定义逐项复现

本次原主控请求的旋转记为 R。用“视觉剪切点减去请求法兰位置”并左乘 R 的转置，得到主控当前补偿隐含的法兰至刀口向量 `[0,0,100] mm`。模型从 SDK 对应 wrist3 原点出发，先沿局部 Z 到 CAD 固定座原点 50.952 mm，再组合刀座旋转和刀口偏移，得到 `[-4.4,-4.9,135.952] mm`。[100 mm 参数](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:68) · [主控补偿](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:1468) · [模型安装链](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/arm_kinematics.py:29)

两套工具位置定义之间的空间差为 36.5502 mm，其中局部 Z 分量相差 35.952 mm。经过当前采摘姿态旋转并加上视觉点误差后，得到模型刀口至场景剪切点约 34.3574 mm。各分量均在 SDK 基座系下计算。

| 误差项 | X / mm | Y / mm | Z / mm |
| --- | --- | --- | --- |
| CAD 刀口相对主控补偿隐含刀口 | +6.10938 | +35.95200 | +2.45875 |
| 视觉剪切点相对场景剪切点 | −3.47721 | −1.79735 | +0.17965 |
| 两项之和 | +2.63217 | +34.15465 | +2.63840 |

SDK FK 对应 `wrist3_Link` 原点，经过 23.5 mm 基座 Z 基准转换后与物理手腕一致。Gazebo 中名为 `flange_link` 的坐标系则位于该原点沿手腕 Z 轴外伸 50.952 mm 处，属于当前 CAD 固定座装配基准。核对工具长度时需要明确使用哪一个原点。[SDK 基准转换](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/arm_kinematics.py:56) · [CAD 固定座基准](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/mujoco_cad.xacro:538) · [刀口 TCP 定义](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro:115)

### 88.42° 的定义与实际刀片网格对照

仲裁器计算的是果梗轴线与刀片平面法向的无向夹角，参考法向取刀座系 +X。夹角接近 0° 时果梗垂直穿过刀面，接近 90° 时果梗几乎平行于刀面。原主控本次输出 RPY `[90°,70°,180°]`，与当前刀座旋转组合后得到参考夹角 88.4162°。[仲裁器定义](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/harvest_physics_arbiter.py:240) · [原主控俯仰设定](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:1416)

用完整 0.2 mm 接触刀片 STL 的大面积三角面法向独立拟合，厚度为 0.200001 mm，刀面法向在刀座系下约为 `[0.99999985,0.00000014,0.00054944]`，距刀座 +X 约 0.0315°。采用这条实物网格法向，本次夹角为 88.3848°，果梗轴与刀面之间约为 1.6152°。将刀片放入 Gazebo 实际左指刚体位姿后，其法向与对应 TF 推算值的最大差为 0.003994°。

在用户描述的初始位姿中，参考刀面倾斜为 1.6300°，完整网格拟合刀面倾斜为 1.5986°，腕部相机高于 TCP 114.0436 mm。初始外观与用户确认一致。初始外观验证和采摘指令对应的刀面方向均已量化。[完整网格与坐标分析](/data/tomato_gazebo/ros2_gazebo_ws/validation/frame_mapping/analyze_mapping.py)

### 当前定位结果

本轮把剩余问题定位到主控补偿使用的工具参考点及姿态约定与当前 CAD 刀口定义之间。单次初始外观可以核验总体装配关系，16 个实际姿态测点进一步确定了仿真内部 SDK 至物理手腕的映射。刀口目标位置与刀面朝向需要分别满足剪切条件。统一旋转所有表示坐标后，点距和交角保持相同，后续适配应依据工具参考点与刀面方向的完整刚体变换。

本轮运动覆盖初始姿态附近的小幅平移和旋转。可视化的采摘目标姿态来自离线几何计算。显示网格采用简化三角面和 0.1 mm 显示量化，测量与法向拟合采用完整网格。真机工具标定的绝对值仍需与真实装配测量对应。
