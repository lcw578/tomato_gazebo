# 底层几何与执行器实测报告

2026-11-20。机械臂安装基准采用用户确认的偏航 180°，底盘采用沿轨 `machine_slide`。本轮修改范围为仿真运动适配器、夹爪适配器、Gazebo 碰撞与驻车插件、刀片传感器引用及验证工具。分支为 `codex/geometry-actuator-validation`。下文保留前轮测量，新增驻车与三相机复验见补充验收章节。

## 驻车与三相机补充验收

本轮采用 v4 原场景、180° 安装、零升降高度、三路 3 Hz RGB-D。侧相机最终补测使用 4000 count/m，腕部使用 1000 count/m。右相机另有 1000 count/m 的通过记录。验证工具查询中继的深度单位，腕部使用曝光时刻的动态手眼 TF。紧凑测量、矩阵、失败对照与源码哈希纳入版本管理。[补充验收证据](/data/tomato_gazebo/ros2_gazebo_ws/docs/evidence/geometry_actuators_2026-11-20.json)

| 相机 | 球心误差 | TCP 指令误差 | TCP 到球面误差 | 动作期间最大驻车位移 | 实际刀片接触 |
| --- | --- | --- | --- | --- | --- |
| 右 | 0.805 mm | 0.010 mm | 0.396 mm | 0.002686 mm | 已检出 |
| 左 | 0.980 mm | 0.007 mm | 0.653 mm | 0.000006 mm | 已检出 |
| 腕部 | 0.573 mm | 0.008 mm | 0.396 mm | 0.003995 mm | 已检出 |

### 驻车根因、修复与对照

原速度控制器通过 GazeboSystem 的 `SetVelocity` 写入速度，零速命令缺少位置保持目标。关闭脚本补偿后，右相机触球测得最大漂移 **10.379223 mm**、TCP 指令误差 **9.899127 mm**、球面误差 **4.944930 mm**，刀片接触为 false，验证按 0.5 mm 驻车门限失败。[失败对照](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/before_sphere_right/measurements.json)

新增插件订阅现有 `/slide_velocity_controller/commands`。零速时在物理更新线程捕获实际关节位置，将 ODE 上下止挡设为该位置，采用 stop_erp=0.8、stop_cfm=1e-10。非零速度时恢复原行程上下限及止挡参数，继续接收速度控制。复位回调恢复原限位并清除锁止状态，以便更新线程重新捕获驻车位置。[slide_parking_brake.cpp 第 16 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/src/slide_parking_brake.cpp:16)、[复位回调第 43 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/src/slide_parking_brake.cpp:43)

触球目标仅在曝光时从相机坐标变换到机械臂底座坐标，后续服务调用使用同一目标。世界坐标用于独立评分，记录中的 `world_target_compensation_enabled=false`、`target_reexpressions_after_midpoint=[]` 可供核查。驻车指标来自动作期间每条接收到的 JointState，包含回初始位、中间点及接近目标的全段最大位移，右、左、腕部各记录 2341、1038、1856 个样本。[目标生成第 277 行](/data/tomato_gazebo/ros2_gazebo_ws/scripts/geometry_actuator_checks.py:277)、[单次目标下发第 313 行](/data/tomato_gazebo/ros2_gazebo_ws/scripts/geometry_actuator_checks.py:313)、[驻车统计第 38 行](/data/tomato_gazebo/ros2_gazebo_ws/scripts/geometry_actuator_checks.py:38)

正向、反向各运行约 2 s，实测速率为 **100.049944、-99.893978 mm/s**。两次停车后均执行 OEM_HOME、验证关节位、OEM_HOME 的机械臂动作，最大漂移均低于 **0.000003 mm**。回筐实测 TCP 目标误差 **0.006228 mm**，动作期间底座最大漂移 **0.003424 mm**。空载夹爪 10 次开合的 20 条服务命令全部成功，最大手指目标残差 **0.001284 mm**。[移动与停车测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/final_rail/measurements.json)、[回筐测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/final_basket/measurements.json)、[夹爪测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/final_gripper/measurements.json)

Gazebo 复位后补测正反向移动与驻车均通过。首次 `/reset_world` 请求执行后发生 DDS 响应超时，后续采用 Gazebo transport 的模型复位入口补测驻车通过，最大位移 **0.000000022 mm**。两条路径的完整日志与实测结果保存在证据目录。[复位后移动测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/after_reset_rail/measurements.json)、[transport 复位后驻车](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/transport_reset_parking/measurements.json)

### 相机业务入口和验收覆盖

真机主控根据 `camera_working_flag` 选择左、右手眼矩阵，并调用切换启动文件。腕部采摘姿态服务使用 `/cam_hand/color/image_rect_raw`、`/cam_hand/aligned_depth_to_color/image_raw` 与 CameraInfo。三路触球分别通过这些中继后的业务话题取图，腕部矩阵按图像时间戳查询并保存。[左右矩阵选择第 1331 行](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:1331)、[相机切换第 2139 行](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/xc_robot_main_control_node.py:2139)、[腕部话题第 27 行](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/xc_robot_main_control/xc_robot_main_control/pick_pose_service_node.py:27)

测试夹具采用直径 8 mm 的静态洋红球、0.5 mm 接近余量，球心、TCP 指令及球面距离采用 3 mm 验收门限。驻车后默认硬接触曾导致关节速度抖动，测试球设置 kp=10000、kd=1、max_vel=0.01、min_depth=0.00001，以柔性接触验证刀片接触反馈。球心真值仅用于生成夹具和独立评分。上述数值为本轮校验工具选定的测试条件。

左相机首个光学坐标测试点 `[0.02, -0.02, 0.35]` m，在 OEM_HOME 到中间点的当前逆解分支中发生前臂、上臂与升降台碰撞，服务返回 30003。接触记录中分别统计到 2664、2027 条对应碰撞，底座最大漂移达到 2.597 mm。最终左侧夹具改为视野内可达点 `[-0.06, -0.12, 0.42]` m，触球通过。右侧使用 `[0.02, -0.02, 0.35]` m，腕部使用 `[0, 0, 0.18]` m。此次覆盖各相机选定单点，原左侧点的碰撞问题仍存在，通用目标轨迹的碰撞避让属于后续完整业务验收范围。[左侧失败测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/left/measurements.json)、[左侧碰撞记录](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/left_failed_contacts.log)

一次三相机启动发生 gzserver 段错误。重试并使用 gdb 观察后启动成功，后续普通启动完成三路及执行器复验。该启动异常的具体触发因素尚待复现，原日志保存在验收目录。[首次启动日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/gazebo.log)、[后续普通启动日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/parking_cameras/gazebo_final.log)

### 补充验收复现入口

以下命令采用独立 ROS 域。启动和检查终端均需设置相同环境，检查命令依次运行。

```bash
cd /data/tomato_gazebo/ros2_gazebo_ws
source /opt/ros/humble/setup.bash
source .overlay/install/local_setup.bash
export TOMATO_SIM_WORKSPACE=$PWD
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=137
export GAZEBO_MASTER_URI=http://127.0.0.1:11737 GAZEBO_IP=127.0.0.1
ros2 launch tomato_sim_gazebo baseline.launch.py left_camera:=true hand_camera:=true remote_depth_units_per_metre:=4000
```

在第二个同环境终端执行。

```bash
python3 scripts/geometry_actuator_checks.py validation/manual/rail --check rail
for camera in right left hand; do
    python3 scripts/geometry_actuator_checks.py "validation/manual/$camera" --check sphere --camera "$camera"
done
python3 scripts/geometry_actuator_checks.py validation/manual/basket --check basket
python3 scripts/geometry_actuator_checks.py validation/manual/gripper --check gripper --cycles 10
python3 -m unittest discover -s tests -v
```

## 驻车修复前的几何与夹爪实测

| 工况 | 回筐 TCP 目标误差 | 夹爪 10 次开合最大手指残差 | RGB-D 球心误差 | TCP 到球面误差 | 刀片接触反馈 |
| --- | --- | --- | --- | --- | --- |
| v4 原场景，升降 0 m | 0.00487 mm | 0.000165 mm | 0.886 mm | 0.159 mm | 已检出 |
| 简化诊断场景，升降 0.5 m | 0.000351 mm | 0.000233 mm | 0.826 mm | 0.148 mm | 已检出 |

v4 测试经 `baseline.launch.py` 启动，使用右相机、3 Hz 相机配置与 1 mm 深度编码。地图源 SHA-256 为 `90c63297014a3a6cff10929fced3fe2f33e3ab9e7faacaba2a6d790e86bef2c3`。回筐服务返回 0，夹爪 20 条开合命令全部成功。完成触球后，经保存的进刀轨迹回退再调用回筐服务，TCP 误差为 0.00479 mm。

原始记录保存在以下文件。

- [v4 启动世界与安装参数](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/v4_world.json)
- [v4 回筐测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/v4_basket/measurements.json)
- [v4 夹爪测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/v4_gripper/measurements.json)
- [v4 球面触碰测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/v4_sphere_contact/measurements.json)
- [v4 触碰后回筐与 TF 导出](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/v4_return_after_touch/measurements.json)
- [升降 0.5 m 回筐测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/acceptance05_basket/measurements.json)
- [升降 0.5 m 球面触碰测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/acceptance05_sphere/measurements.json)
- [升降 0.5 m 触碰后回筐测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/acceptance05_return_after_touch/measurements.json)
- [升降 0.5 m 夹爪测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/acceptance05_gripper/measurements.json)

## 1.07 m 偏移的来源与修复

旧固定回筐关节角在当前装配下，经独立 FK 计算得到筐坐标内 TCP 为 `[1.069999999, -0.011999957, -0.219999012]` m。相对原释放目标 `[0, 0, -0.22]` m 的三维误差为 **1070.067 mm**。旧关节目标与当前 180° 装配基准不匹配。当前装配几何决定了新的回筐关节目标。[根因计算记录](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/root_causes.json)

| 变换 | 平移，单位 m | RPY，单位 rad |
| --- | --- | --- |
| `lift_platform → arm_base_link` | `[0.002, 0.004, -0.704]` | `[π, 0, π]` |
| `chassis_cad_datum → lift_platform` | `[0, 0, -lift_height]` | `[0, 0, 0]` |
| `chassis_cad_datum → basket_link` | `[-0.533, 0.010, -0.880]` | `[0, 0, 0]` |
| `cutter_base_link → tool_tcp` | `[-0.0049, -0.0044, -0.085]` | `[0, 0, 0]` |
| `basket_link → arm_base_link`，升降 0 m | `[0.535, -0.006, 0.176]` | `[π, 0, π]` |
| `arm_base_link → tool_tcp`，v4 回筐实测 | `[0.534997, 0.006002, 0.395997]` | `[2.094392, -0.785407, -1.884944]` |

URDF 的安装定义见 [tomato_picker_full.urdf.xacro 第 73 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro:73)、[第 82 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro:82)和[第 86 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro:86)。TCP 的变换链包含腕部到法兰 50.952 mm、法兰到刀具的轴向转换及刀尖偏置。[arm_kinematics.py 第 31 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/arm_kinematics.py:31)

仿真 `/return_to_box` 现在读取 `arm_base_link ← basket_link` 的实时 TF，再求解回筐 IK。已有显式 `box_joints_rad` 配置仍可提供自定义六关节目标。[sim_aubo_motion_service.py 第 266 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_aubo_motion_service.py:266)

升降 0.5 m 时，原来固定的 `basket Z=-0.22 m` 目标进入升降台下方。复测记录包含前臂与升降台碰撞，轨迹完成后实际关节持续振动，并有一次出现 NaN。因此释放高度采用 `min(-0.22, arm_base_in_basket.z - 0.20)`，得到零升降 `-0.22 m`、0.5 m 升降 `-0.524 m`。IK 同时比较两个腕部解，选择距回退零位较近的一支，避免腕部约 4.91 rad 的绕转。[arm_kinematics.py 第 80 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/arm_kinematics.py:80)、[碰撞采样](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/basket05_contacts.txt)

上述释放高度和腕部解选择是本轮采用的仿真实施方案。筐口内侧范围约为 `|X|<0.15 m`、`|Y|<0.205 m`。以 TCP 为半径 35 mm 果球的中心，v4 回筐时水平边缘余量约为 **115 mm、170 mm**。该结果验证了空载 TCP 位于筐口上方，携果偏置、释放速度与落果轨迹属于后续携果入筐验收项。

## 夹爪 1.9 mm 偏差的来源与修复

真机节点将开、合目标配置为 **600、2702 脉冲**，声明 `position_tolerance=5`。当前执行路径调用 485 驱动后，直接把驱动返回的 `success` 写入服务响应。485 驱动发送目标后读取位置，收到有效位置响应即返回成功。参数 `position_tolerance` 的引用位于声明、读取和日志输出处，闭合流程采用位置读回成功判定。[gripper_control_node.py 第 28 行](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/gripper_driver_v2/gripper_driver_v2/gripper_control_node.py:28)、[第 110 行](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/gripper_driver_v2/gripper_driver_v2/gripper_control_node.py:110)、[gripper_485_driver.py 第 170 行](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/gripper_driver_v2/gripper_driver_v2/gripper_485_driver.py:170)、[第 227 行](/data/tomato_gazebo/robot_ws_fixed-v1.0-master/src/gripper_driver_v2/gripper_driver_v2/gripper_485_driver.py:227)

仿真目标主齿轮为 `0.4 rad`，左右手指 mimic 比例为 `-0.020999`、`-0.020995 m/rad`，对应闭合目标 **-8.3996 mm、-8.3980 mm**。原始接触记录反复出现右侧简化刀片与左侧精细 CAD 刀片相互接触。参考 MJCF 的接触配置对左右手指具有排除定义。[URDF 第 146 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro:146)、[原始接触记录第 121772 行](/data/tomato_gazebo/ros2_gazebo_ws/validation/current_review_20261120T114732_Aym03e/motion/gazebo_contacts.log:121772)、[参考 MJCF 第 848 行](/data/tomato_gazebo/assets/fk_reference/tomato_picker_standalone/tomato_picker_standalone.xml:848)

修复前追加的三次空载闭合测量中，手指目标误差达到 **1.85–2.79 mm**，重复开合后的闭合结果变化明显。新增模型插件为左右手指分配碰撞类别，排除这两个链节之间的内部碰撞。[修复前测量](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/gripper_before/measurements.json)、[gripper_collision_filter.cpp 第 10 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_gazebo/src/gripper_collision_filter.cpp:10)

同组执行器设置下，修复后 10 次开合残差降至微米以下，并检出了外部探针和标定球与刀片的实际接触。这组对照将主要原因定位到刀片内部碰撞干涉。仿真夹爪服务现在同时检查主齿轮误差 `<0.002 rad` 和两手指误差各 `<0.4 mm`，成功反馈同时受实际手指位置约束。[sim_gripper_node.py 第 19 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_gripper_node.py:19)、[外部探针接触记录](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/external_contact_lift05/measurements.json)

另一个已修复的问题是刀片传感器引用了转换前的碰撞体名称。Gazebo 的 URDF 转换给碰撞体添加了固定关节合并前缀和序号，原传感器只发布空接触状态。现在引用实际 SDF 名称，并有转换回归测试核对该碰撞体存在。[URDF 第 163 行](/data/tomato_gazebo/ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro:163)、[test_blade_contact_sensor.py 第 12 行](/data/tomato_gazebo/ros2_gazebo_ws/tests/test_blade_contact_sensor.py:12)

## RGB-D 到 TCP 触球验证

验证使用直径 **8 mm** 的静态洋红色标定球，使球体可以进入刀尖附近的狭窄空间。球心由同时间戳 RGB、对齐深度和 CameraInfo 反投影后拟合得到。命令坐标经过保存的 `T_arm_base_from_optical` 矩阵转换，Gazebo 球心真值用于独立评分。[rgbd_sphere.py 第 7 行](/data/tomato_gazebo/ros2_gazebo_ws/scripts/rgbd_sphere.py:7)、[geometry_actuator_checks.py](/data/tomato_gazebo/ros2_gazebo_ws/scripts/geometry_actuator_checks.py)

右相机手眼矩阵来自当前仿真装配 TF，采用现有标定文件格式。测试选择 **3 mm** 作为球心、TCP 指令及球面距离的临时验收门限。深度编码单位为 1 mm，触碰目标额外沿接近方向推进 **0.5 mm**，以获得可观测接触。v4 最终记录中，球面拟合 RMS 为 0.243 mm，球心误差 0.886 mm，TCP 指令误差 0.888 mm，独立 Gazebo 腕部位姿计算出的 TCP 到球面误差 0.159 mm，TF TCP 与物理 TCP 的差为 0.0374 mm，并检出精细刀片接触球体。

这组早期测量中，底座沿轨平移约 **9.30 mm**。当时验证脚本将图像时刻的目标保存到世界坐标，并在中间点后依据最新底座 TF 重新表达目标。本轮补充验收已去掉该目标补偿，采用驻车约束及三相机分别复测，实测数据见补充验收章节。

RGB 图像、原始深度数组及手眼矩阵均随测量保存。[RGB 图](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/v4_sphere_contact/sphere_rgb.png)、[深度数组](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/v4_sphere_contact/sphere_depth.npz)、[右相机矩阵](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/v4_sphere_contact/calibration/right/calibration_results.txt)

## 构建、回归与复现

本轮构建成功完成 9 个包。10 个回归测试全部通过，覆盖配置、两种升降高度的回筐解、夹爪到位判定、量化深度球面拟合和刀片传感器引用。模型结构检查通过 34 个网格引用、10 个轮关节、三路 RGB-D 定义及轨道约束。运行时五个控制器均为 active。[构建日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/build_final.log)、[测试日志](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/unit_tests_final.log)、[结构检查](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/structure.log)、[控制器检查](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/controllers.log)

真机主控入口、主控节点、YOLO 服务三个文件的 SHA-256 均匹配本轮开始使用的审计工作区参考值。完整单果验收仍需实际完成识别果梗、进刀剪离、携果回退、回筐释放和果实稳定落筐。[源码参考校验](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/root_causes.json)

在已启动且环境变量一致的仿真终端，可以按下列方式复测。各检查依次执行以保持机械臂命令顺序。

```bash
cd /data/tomato_gazebo/ros2_gazebo_ws
source /opt/ros/humble/setup.bash
source .overlay/install/local_setup.bash
python3 scripts/geometry_actuator_checks.py /data/tomato_gazebo/ros2_gazebo_ws/validation/manual/basket --check basket
python3 scripts/geometry_actuator_checks.py /data/tomato_gazebo/ros2_gazebo_ws/validation/manual/gripper --check gripper --cycles 10
python3 scripts/geometry_actuator_checks.py /data/tomato_gazebo/ros2_gazebo_ws/validation/manual/sphere --check sphere
python3 -m unittest discover -s tests -v
```

## 本轮开始前的工作区与基线整理

首次阶段一修复后的工作区包含 19 个已跟踪修改文件和 9 个新增文件，当时的路径清单保存在验收目录。[当前未提交改动清单](/data/tomato_gazebo/ros2_gazebo_ws/validation/geometry_actuators/git_status_final.txt)

开始实施时共有 15 个已跟踪文件存在未提交修改，主要涉及独立轮组、轮速转发、结构检查和依赖说明。路径以仓库根目录为基准。

```text
README.md
ros2_gazebo_ws/README.md
ros2_gazebo_ws/docs/dependencies.md
ros2_gazebo_ws/scripts/check_controllers.py
ros2_gazebo_ws/scripts/export_mujoco_geometry.py
ros2_gazebo_ws/scripts/phase1_structure.py
ros2_gazebo_ws/scripts/phase3_checks.py
ros2_gazebo_ws/scripts/prepare_overlay.py
ros2_gazebo_ws/src/tomato_sim_adapters/tomato_sim_adapters/sim_agv_control_node.py
ros2_gazebo_ws/src/tomato_sim_description/config/controllers.yaml
ros2_gazebo_ws/src/tomato_sim_description/urdf/mujoco_cad.xacro
ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_control.xacro
ros2_gazebo_ws/src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro
ros2_gazebo_ws/src/tomato_sim_gazebo/launch/phase1.launch.py
ros2_gazebo_ws/verify_phase1.sh
```

轮组网格重建入口如下。脚本从 `/data/tomato_gazebo/assets/fk_reference/tomato_picker_standalone/tomato_picker_standalone.xml` 读取 MJCF 与原始网格，使用 NumPy、SciPy、trimesh 生成描述包网格和 `mujoco_cad.xacro`，包含两个主动轮和八个从动轮的定义。运行会覆盖生成文件，适合在检查差异后使用。[export_mujoco_geometry.py 第 12 行](/data/tomato_gazebo/ros2_gazebo_ws/scripts/export_mujoco_geometry.py:12)

```bash
cd /data/tomato_gazebo/ros2_gazebo_ws
.venv_yolo/bin/python scripts/export_mujoco_geometry.py
./build_overlay.sh
git diff --check
```

基线按两组提交整理。第一组收录上述 15 个文件的轮组接入及对应检查，第二组收录回筐、夹爪、驻车和三相机验收。两个 URDF 文件的混合差异已按改动来源拆分。生成网格、运行世界、图像和完整测量数据保存在 `/data` 的现有忽略目录，紧凑实测证据随第二组提交纳入版本管理。独立审查以参考资产在内存中重建了 26 个宏与 19 个 STL，结果与工作区逐字节一致。
