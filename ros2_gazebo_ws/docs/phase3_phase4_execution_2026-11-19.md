# 历史记录 · Phase 3 / Phase 4 执行记录与后续联调说明

> **历史记录。** 本页描述旧单株场景的早期试跑，含当时的包装器做法与失败数据。用户随后指定 `tomato_corridor_v4.world` 并确认机械臂 180° 安装；当前做法与验证结果以 [v4 仿真基线与采摘迁移](v4_baseline_2026-11-20.md) 为准。

日期：2026-11-19  
工程目录：`/data/robot_assembly/ros2_gazebo_ws`  
状态：实现已搭建；Phase 3 与 Phase 4 端到端验收均为 FAIL，详见 `validation/phase3_*` 与 `validation/phase4_*`。

## 1. 本轮目标与证据边界

本轮工作按用户的 Phase 3/4 修正要求，以 2026-11-16 MuJoCo 已验收链路为规则来源。现有 ROS 2 主控和 `yolov8_service` 继续使用真机工作空间源码；Gazebo 侧用适配器替代 AUBO/夹爪/相机硬件。没有修改 `/home/lcw/robot_ws_fixed-v1.0-master` 真机源码，也没有修改 `/data/robot_assembly/gazebo_migration` 原始场景。验证生成物位于本工作区 `validation/`。

参考证据为：`/data/robot_assembly/docs/current_pick_baseline_2026-11-16.md`、`/data/robot_assembly/docs/f0_f2_acceptance_2026-11-16/report.md` 及 `validation.json`、`two_visible_queue.json`，旧版模型 `/data/robot_assembly/model/tomato_picker_standalone/tomato_picker_standalone.xml`，迁移源场景 `/data/robot_assembly/gazebo_migration/single_plant_verify.world` 与果串真值 `truss_ground_truth.json`。报告记录两次独立双串物理运行均为剪断 2、入筐 2、主茎接触 0；它是 MuJoCo 基线，不是本次 Gazebo 验收结果。

## 2. MuJoCo 规则审计结果

### 解耦边界

果串根部果梗与番茄串在旧模型中属于同一果串刚体；主茎和果串之间由植株模型的固定连接构成。采摘边界不是把整串果实从视觉上隐藏，而是把目标果梗沿指定切平面拆成母株侧与果串侧几何，剪切成功后才改变物理所有权。

Gazebo 生成器在 `scripts/prepare_phase3_world.py` 中按 `truss_ground_truth.json` 的剪切中心和果梗单位切向计算平面。圆柱梗段的两个端点投影到该平面法向：完全落在果串侧的段及 cap 转移到果串；跨面的段在交点处分为两段并分别保留在母株和果串侧；完全母株侧不转移。剪切边界关节球按球半径是否跨过平面沿用 MuJoCo 的移除规则。生成的植株实体只保留主茎 `g_v9_stem*` 和目标剪切段的碰撞，视觉保持原样；其余 590 个非目标碰撞代理在本次单串验证模型中移除。四个果串成为独立顶层 Gazebo entity。它们起始为 `<static>false>`、`<gravity>false>`、`<kinematic>true>`，从而保留 RGB/深度画面，同时可由状态服务更新位置。

### TCP-剪切点与实际刀口判定

旧版采摘点的 TCP 参考是 `contact_tcp`，Gazebo 对应 `tool_tcp`，不是 `wrist3_Link` 法兰中心。位置误差使用三维欧氏距离：

```text
e_tcp = || p_tool_tcp(world) - p_cut_ground_truth(world) ||_2
接受条件：e_tcp <= 0.005 m
```

5 mm 是最终接近与剪切仲裁阈值；旧记录中的姿态接近容差为 0.03 rad。它们不能替代实际刀口碰撞条件。剪切还必须同时满足：Gazebo 接触对包含从 MuJoCo CAD 复制的精确 0.2 mm 刀片 `mujoco_contact_blade` 和目标分离梗 `contact_stalk_*`；接触点在 cutter frame 的闭合刀槽内部，`-0.0075 < y < 0.0025 m`、`-0.094 < z < -0.052 m`；刀片接触点与闭合刀片面 `x=-0.0048973261 m` 的误差不超过 0.5 mm；目标梗方向相对刀刃法向满足旧版自然进刀夹角上限 30°。几何接触、槽内位置、TCP 距离、进刀方向和主茎无接触是合取条件，不能用单一距离规则替代。

净空参数也分开记录：规划阶段主茎最小净空 5 mm，执行阶段 3 mm，接触报告上限 8 mm。它们是旧版规划/监测边界；本轮 Gazebo 的“主茎接触 = 0”验收按真实 contact sensor 消息计数，不会将某个假定净空值当作零接触。

### 主茎接触计数

主茎传感器覆盖 `g_v9_stem*` 碰撞。只要 contact pair 一端属于 `tomato_picker::`，当前活动接触周期计一次，持续接触期间不会每帧重复累计；离开再触碰会开始新周期。机器人本体、刀爪与携果过程的主茎接触都记录碰撞对到 `validation/phase3_runtime.json`。本轮 Phase 3 第一次试跑曾记录 8 个接触周期，碰撞对涉及腕部、前臂、上臂和刀爪；该运行从轨道 0 位驶向工位时发生，且机械臂归位失败。之后将固定工位出生轨位改为 0.3 m，试跑中主茎接触回到 0。Phase 4 当前运行发生了 1 个主茎接触周期，因此不满足验收要求。

### 持果与入筐

剪断后保存果串相对 `tool_tcp` 的刚体位姿：`T_tcp_fruit = inverse(T_world_tcp_at_grasp) * T_world_fruit_at_grasp`。持果期间每次根据最新 TCP 计算 `T_world_fruit = T_world_tcp * T_tcp_fruit`，通过 `/set_entity_state` 更新原独立果串实体，因此相机中果串应随 TCP 移走、原位置随之消失。

开爪释放时取当前果串世界位姿，删除其运动学实体，再从相同 SDF 以同名 spawn 为动态、带重力实体；由 Gazebo 重力将其落入 `basket_link`。自动检查把释放后果串变换到 `basket_link` 局部坐标，验证筐口范围 `|x|<0.16 m`、`|y|<0.21 m`、`0.10<z<0.32 m`。RGB 与深度原位变化分别检查；视觉残影门禁要求两个模态在原果串投影 ROI 中均出现大于 10% 的变化。

## 3. Phase 3 实现面

主要实现：

- `scripts/prepare_phase3_world.py`：复制场景到工作区、将单株工位 Y 对齐至 -0.475 m、按真值切分目标果梗，生成静态母株和四个独立果串 SDF。
- `src/tomato_sim_adapters/tomato_sim_adapters/harvest_physics_arbiter.py`：订阅 `/midpoint_reached`、进刀阶段、真实刀口/主茎接触和夹爪开闭；发布 X1 与 `test_result`；在真实刀口接触并满足所有旧规则后才允许剪断、持果和释放。
- `src/tomato_sim_description/urdf/tomato_picker_full.urdf.xacro` 与 `src/tomato_sim_description/urdf/mujoco_cad.xacro`：真机整机 CAD、原刀片接触网格、`tool_tcp` 帧、腕部及 RGB-D 安装。
- `src/tomato_sim_gazebo/src/initial_joint_state_plugin.cpp`：模型插入时尝试设置物理初始轨位与六轴姿态；控制器启动后仍需验证关节能否保持。
- `scripts/phase3_checks.py`、`verify_phase3.sh`：负例、正例、相机变化、跟随、释放和主茎碰撞验收。

X1 只在 `/midpoint_reached` 已到、已进刀、TCP 与 GT 剪切点误差不超过 5 mm 且未碰主茎时置位。满足条件时发布 `test_result=2`；主茎接触发布 `test_result=3` 并清 X1。夹爪 close 后仍需看到真正的刀片-目标梗接触才会 `CUT_SUCCESS`。距离误差负例不应触发 X1 或 detach；主茎负例应回退结果 3。

## 4. Phase 4 坐标链与真机代码接入

`prepare_phase4_calibration.py` 从当前 TF 树读取 `arm_base_link <- cam_left/right_color_optical_frame`，以标准 4x4 齐次矩阵写入 `validation/phase4/calibration/{left,right}/calibration_results.txt`。当前左相机矩阵为：

```text
[[ 0.9702957, 0, -0.2419219,  0.358341 ],
 [ 0.2419219, 0,  0.9702957,  0.040530 ],
 [ 0,        -1,  0,          0.445732 ],
 [ 0,         0,  0,          1        ]]
```

相机内参来自 Gazebo CameraInfo，采摘点像素与米制深度先按针孔模型反投影，再乘 `T_arm_base_from_optical`。仿真包装器 `phase4_main_control.py` 用仿真 TF 标定路径替代源码中固定的真机文件路径，禁用 MoGe，设 `is_real_camera_data=False`、`depth_scale=0.00025`，与相机转发器把米制深度放大 4000 写入 `16UC1` 的编码相匹配。它将 `gripper_length` 设为 0，使主控发给 `/move_to_pose` 的目标以实测刀尖 TCP 为基准；否则真机 0.10 m 末端偏置会让“法兰到点”和“刀尖到点”混淆。真机源码仍保留原值。

`ros_vision_compat.py` 给 Conda 的 NumPy 2/OpenCV 环境提供 NumPy 实现的 CvBridge 图像转换与针孔 deprojection shim，绕过系统 cv_bridge NumPy ABI 不兼容和本机无 `pyrealsense2` 的问题。MoGe 类使用“禁用即报错”的占位，不加载缺失的 MoGe checkpoint 或 huggingface 依赖。权重沿用现有 YOLOv8 分割模型：`tomato-seg2025063018-yolov8-xucan.pt`，类别为 `stem`、`tomato`、`trunk`；workspace 的 `best.pt` 指向该模型。

## 5. 本轮实测与失败门禁

### 构建/启动

执行 `source /opt/ros/humble/setup.bash && colcon build --base-paths src --symlink-install`，9 个 ROS 包构建成功。`python3 -m py_compile`、`bash -n verify_phase3.sh verify_phase4.sh`、`git diff --check` 均通过。生成场景 `gz sdf -k` 通过。RGB 与深度 relay、CameraInfo、主控 `/yolo_detection` 服务以及 Gazebo `move_to_pose`/夹爪服务均可发现。

### Phase 3

`verify_phase3.sh` 最终 `FAIL`，状态文件为 `validation/phase3_status.txt`。上一次完整运行摘要 `validation/phase3_summary.json` 中，轨道实测起点约 0.304 m，距 0.300 m 目标 4.1 mm，落在为当前试验设置的 50 mm 固定工位容差内；主茎接触在直接工位运行中为 0。机器人六轴读数却偏离 OEM home，最大关节误差约 1.425 rad（`wrist2_joint` 约 0.087 rad，期望约 1.512 rad）。验收在初始姿态门禁停止，未运行错位负例、剪离、持果、入筐和视觉原位消失检查。因而当前不能判 Phase 3 DoD 通过。

最初把 Gazebo 物理初态全交给 ros2_control `<initial_value>` 未生效。当前加入 Gazebo ModelPlugin 并确认日志打印 “Initialized physical joint positions”，但控制器激活后的实测关节仍偏离；要继续排查物理自碰/重力/控制器初态接管顺序。修复前不要放宽 0.08 rad 初态门限，也不要在验收中直接 teleport TCP 代替真实 trajectory。

### Phase 4

`validation/phase4_status.txt` 当前为 `FAIL`。`verify_phase4.sh` 实测生成了左右手眼矩阵；Conda 中 `torch 2.5.1+cu121`、CUDA 可用、Ultralytics 8.4.179；真实 YOLOv8 服务加载模型并报告已移到 `cuda:0`。主控节点启动、读到相机 CameraInfo/矩阵并进入图像回调。Gazebo RGB `/sim/cam_left/image_raw` 与转发 `/camera/color/image_raw` 都实际收到了 848×480 `rgb8` 图像，CameraInfo 为 `fx=fy≈605.535`、主点 `(424.5, 240.5)`。

480 秒有界窗口已结束。`validation/phase4_summary.json` 为 `passed:false`，运行状态仍是 `ATTACHED_TO_STEM`，无视觉点、`CUT_SUCCESS`、X1、`test_result=2` 或释放记录；主茎接触计数为 1。主控日志停在首次图像回调，YOLO 服务节点持续运行但没有形成 Detection2UI。即使 RGB、深度和 CameraInfo 均已到达，Phase 4 闭环仍未开始，不能标记通过。

一个待定位的 runtime 问题是首次服务请求阶段停滞：`yolov8_service` 和真机主控均互相发现 `/yolo_detection`，但主控只输出一次“进入图像回调”，YOLO 节点没有请求/响应日志，主控也没发布 `/detection2ui`。需给请求、推理开始/结束、结果数量和主控回调各加外部适配层计时/状态观测，再区分 GPU 推理、响应转换、服务回调和主控图像处理。不要先修改真机 YOLO/主控算法源码来绕过这条链路。

## 6. 复跑步骤与下一个门禁

```bash
cd /data/robot_assembly/ros2_gazebo_ws
./verify_phase3.sh
./verify_phase4.sh
```

两脚本都会在退出时写 `PASS`/`FAIL`。Phase 4 使用 ROS domain 92、Phase 3 使用 domain 91；有占用时设置 `ROS_DOMAIN_ID` 与 `GAZEBO_PORT`。Phase 4 总有界时间可用 `PHASE4_TIMEOUT_S` 调整，但延长时间不能替代阶段成功条件。

继续顺序：首先修复初始关节失稳，实测静止 2 秒内最大关节误差与速度稳定门限；然后只跑已知 TCP 的到位/回退小测且主茎接触 0；再跑刀口/梗接触负例和正例；其后查 YOLO 服务首个请求耗时与消息链；最后才重跑真机主控从检测到 `/move_to_pose`、X1/反馈、close、携果 return、open 到复位的闭环。每一阶段都在 JSON 保留请求坐标、`tool_tcp` GT 偏差（mm）、反馈、接触周期和耗时。完成视觉验证时，同时做真实 RGB/深度图像像素/ROI 差分，单凭 entity pose 变化不能证明渲染刷新。

## 7. 当前产物

- 迁移总研究：`/data/robot_assembly/docs/ros2_gazebo_migration_study_2026-11-19.md`
- Phase 3 场景/真值：`validation/phase3/phase3_scene.json`
- Phase 3 摘要/状态/运行时：`validation/phase3_summary.json`、`validation/phase3_status.txt`、`validation/phase3_runtime.json`
- Phase 4 左右外参：`validation/phase4/calibration/left/calibration_results.txt`、`right/calibration_results.txt`
- Phase 4 摘要/状态/节点日志：`validation/phase4_summary.json`、`validation/phase4_status.txt`、`validation/phase4_main_control.log`、`validation/phase4_yolo.log`
