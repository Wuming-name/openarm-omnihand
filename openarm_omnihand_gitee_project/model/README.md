# OpenArm + OmniHand portable articulated model 1.0.0

离线模型资产：OpenArm v10 七轴保持原定义，OmniHand SDK 0.8.0 的右手恢复为 **16 个转动关节（10 独立 + 6 mimic）**。全模型 36 links、35 joints、23 转动关节、17 独立自由度。默认是已验收的收拢拇指姿态，四指轻微弯曲；`closed_hand` 并非完全握拳。没有真实硬件插件或 CAN 依赖。

## ROS 2 Humble / RViz

依赖 Ubuntu 22.04、ROS 2 Humble、ament/colcon、robot_state_publisher、joint_state_publisher_gui、RViz、Xacro；检查工具另需系统 numpy/scipy/PyKDL。运行位置可以任意移动，不要同时 source 旧的同名描述包。

```bash
cd /任意位置/openarm_omnihand_portable_model
bash tools/build.sh
bash tools/display.sh
```

默认打开 RViz 和 Joint State Publisher GUI。滑动 `omni_R_thumb_mcp_joint`，PIP/DIP 自动遵循 1.33/1.3 倍联动；四指 DIP 遵循对应 PIP 的 1.097 倍。GUI 只显示 17 个独立滑块，中指 ABAD 和所有安装、指尖固定连接不设滑块。GUI 不是避碰控制器；任意拖动不保证无碰撞。只对交付示例做了有限检查。

等价标准启动：

```bash
source /opt/ros/humble/setup.bash
source runtime/install/local_setup.bash
ros2 launch openarm_omnihand_description display.launch.py
```

该 launch 仅启动描述、状态发布、RViz。`tools/display.sh` 额外安装继承的 AF_CAN 禁止过滤器，建议使用。Ctrl-C 正常关闭。默认 ROS_DOMAIN_ID=188，可在启动前覆盖。

精确示例（与 GUI 二选一，确保只有一个 JointState 发布者）：

```bash
bash tools/display.sh gui:=false
# 另一个终端，进入同一目录：
bash tools/set_pose.sh intermediate_hand
bash tools/set_pose.sh open_hand
bash tools/set_pose.sh closed_hand
```

每次过渡 8 秒。局部视角：`bash tools/display.sh gui:=false view:="$PWD/ros2_description/rviz/hand.rviz"`。
标准 `/joint_states` 和完整 TF 由 robot_state_publisher 处理；示例输入明确是 `SYNTHETIC_MODEL_POSE`。没有速度测量或硬件反馈含义。

## 模型文件与定义

- `ros2_description/urdf/openarm_omnihand.urdf`：完整主描述，包内 package:// 网格。
- `ros2_description/xacro/openarm_omnihand.urdf.xacro`：包含主 URDF，不重复维护模型。
- `manifest/joint_manifest.json`：全部固定/可动/mimic 关节、轴、原点、限位、名称顺序。
- `ros2_description/config/poses.json` 和 `examples/*/joint_states.yaml`：收拢、中间、张开三个候选。
- `ros2_description/config/effective_limits.json`：考虑从动关节限位和速度上限后的独立关节可用范围。原始 MCP 上限×1.33 略超过 PIP 上限，因此 GUI 使用数学交集作为显示软限位，URDF 原始限位未改。GUI 的零增益 safety_controller 只是范围元数据，不是电机安全控制器。
- `ros2_description/config/openarm_omnihand.srdf`：原 right_arm 与新增 hand 分组；31 组原碰撞豁免保持不变。它不是已验收的全手动态规划配置。
- `sources/`：仅用于来源和重建的冻结证据，不能把其中原始 URDF 当作入口。

`open_hand` 是官方零关节角的伸展候选，侧摆为零，不声称最大外展。中间姿态是收拢与伸展的线性中点。七轴示例均为明确合成值 `[0.15,0.15,-0.15,0.15,0.1,-0.1,0.1]` rad，不是假冒真实当前姿态。

## Isaac Sim

交付 `simulation/usd/openarm_omnihand.usda`，几何内嵌，无外部 USD 层。24 刚体、23 转动关节、6 个 PhysX mimic 约束。初始 PhysicsJointState 和可视姿态均为收拢手型。

实际测试版本 **6.0.1-rc.7+release.42383.32955d8d.gl**；本机 Isaac Lab 3.0.0 存在，本包使用 Isaac Sim API，未声称完成 RL 任务集成。

```bash
ISAAC_SIM_PATH=/isaac-sim bash tools/isaac.sh --gui
# 独立 headless 模式：
ISAAC_SIM_PATH=/isaac-sim bash tools/isaac.sh
```

需已安装可运行的 Isaac Sim、对应 GPU/驱动和显示环境。运行器默认按闭合→中间→张开→闭合进行有限的零重力驱动演示、保存实际积分状态及截图后退出。真实 PhysX 关节目标驱动；不是 Xform 伪运动。24 秒结果只证明该模型的有限仿真可运行性，质量/惯量和接触包络未经实物动力学标定。

已有本机容器用法（将 `p` 换成容器能看到的本包路径）：

```bash
p=/workspace/local_projects/OPENARM_OMNIHAND_PORTABLE_MODEL_20261025_173300
docker exec isaac-lab-base-clabdev bash "$p/tools/container_run.sh" "$p/tools/isaac_demo.py" --gui
```

仿真映射按关节名称执行：PhysX 的 BFS DOF 顺序与 URDF 不同，不能按数组下标直接混用。运行记录 `reports/isaac_result.json` 给出实际顺序。独立输入 17 维，完整状态 23 维，6 个从动关节由引擎约束，不新增驱动器。

重建 USD：先保留旧输出，运行 `tools/prepare_platform_data.py`，再用 Isaac Python 运行 `tools/build_usd.py`。构建器拒绝覆盖已有 USD；已附带可验证的网格 NPZ，转换不依赖旧工程工具或旧 USD。表面三角形相同，SDF 521/6 的烘焙仍存在原有包络认证限制。

## MuJoCo 与 Gazebo

`simulation/mjcf/openarm_omnihand.xml` 在 MuJoCo 3.4.0 已实际加载、FK核对并渲染三个手型；23 坐标、17 个位置执行器、6 个 mimic equality。使用命名 `keyframe` 初始化（`closed_hand`/`intermediate_hand`/`open_hand`）。

```bash
MUJOCO_GL=egl python3 tools/no_can.py python3 tools/mujoco_check.py
```

**MuJoCo 动力学未通过**：网格凸包在 link5/link7 产生约 8 mm 交叠，检查程序在积分前拒绝。没有删除碰撞体、增添豁免或冒充物理通过；MJCF 适用于有限运动学/渲染检查，不能直接当作训练就绪动力学模型。
Gazebo 未生成 SDF、未实际运行；本机未发现 gz/ign 可执行入口。标准 URDF 并不等于 Gazebo 验收通过。

## 复核、重建及打包

```bash
PYTHONNOUSERSITE=1 python3 tools/validate_model.py
PYTHONNOUSERSITE=1 python3 tools/validate_joints.py
PYTHONNOUSERSITE=1 python3 tools/assemble_model.py  # 包内源文件重建一致性，不覆盖
bash tools/check_collision.sh                    # 需 MoveIt 2 开发包，有限离线检查
python3 tools/verify_hashes.py                    # 应在运行会改写报告的工具前执行
bash tools/package_model.sh
```

`runtime/` 是本地构建/试验缓存，未入包。ZIP/TAR.GZ 使用同一内容集；`manifest/asset_hashes.json` 校验包内文件（自身除外）。最终压缩包 SHA256 在旁置 `dist/archive_receipt.json` 与最终报告，避免将压缩包自身哈希循环嵌入压缩包。主报告作为旁置交付，包内 `reports/validation_summary.md` 提供相同验收说明。

来源、许可和限制见 `LICENSES/`。OpenArm 为 Apache-2.0，选用 OmniHand SDK 网格为 MulanPSL-2.0；本机派生安装网格的独立再分发许可仍未知。本包按现有本机资产整理，不声称所有资产可统一重新授权或公开再分发。
