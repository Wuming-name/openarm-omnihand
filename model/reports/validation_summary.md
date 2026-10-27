# OPENARM-OMNIHAND-PORTABLE-ARTICULATED-MODEL 验收报告

结论：**可迁移的可动模型、ROS 2/RViz 实际可视化和有限 Isaac PhysX 演示已完成；存在明确的碰撞、动力学与再分发许可限制。** 本任务没有真实 CAN、真实手指或机械臂动作。

## 1. 来源选择及版本

独立工作区：`/home/yh/isaaclab-dev/local_projects/OPENARM_OMNIHAND_PORTABLE_MODEL_20261025_173300`，资产包版本 1.0.0。

| 来源 | 用途与判定 |
|---|---|
| MOTION_MVP_CLOSED_HAND_PHYSX_20261024_163123 | 已验收收拢手型的组合 URDF、视觉/碰撞网格、惯量、安装/TCP 变换；沿用源三角面转换数据，不复制旧固定手型 USD 充当可动资产 |
| MOTION_MVP_OMNIHAND_CLOSED_POSE_20261024_160100 | 收拢姿态验收和可复用离线 MoveIt 碰撞核验工具 |
| MOTION_MVP_E2E_READINESS_20261024_233042 | 工程背景/既有模型体系；没有恢复被取消的部署收尾任务，也没有复制真实控制后端 |
| OmniHand SDK 2025 / 0.8.0 本机冻结快照 | 16 个可动关节的原点、轴、限位、类型和 6 个 mimic 定义；其尺寸与既有网格匹配，全部 16 项定义逐字段相等 |
| OmniHand description 2026.2.1 | 对比发现拇指与四指尺寸、限位变化，不混用新版运动学与旧网格 |
| OpenArm description v10 | 历史官方来源记录：https://github.com/enactic/openarm_description ，commit `73ca62f3e4d04f61d2d1efa112927fe763d4b1f1`。来源登记在 `sources/upstream_provenance.json`；本轮不声称重新在线认证 |

OmniHand SDK README 明确“10 active + 6 passive”。该冻结快照没有可核验 Git commit，实际文件 SHA256 是本交付的身份依据。没有把模型定义等同于当前实物标定。

## 2. 交付结构

`ros2_description/` 是标准 ament 描述包（URDF、Xacro、完整网格、配置、launch、RViz 视角、状态示例播放器）；`simulation/usd/` 是可独立加载的 23 关节 USD 及可重建网格数据；`simulation/mjcf/` 包含 MJCF、相对路径 OBJ 和映射。`examples/`、`tools/`、`tests/`、`LICENSES/`、`manifest/`、`reports/`、`screenshots/` 提供状态、复现、来源、检查与实际截图。

`runtime/` 构建缓存、初步试验产物和桌面原始截图不入包。`sources/` 是审计/重建证据，保留原始 URI，不是运行入口。两个压缩包使用相同文件集；最终报告及压缩包自身哈希旁置，包内保留本验收说明。

## 3. 关节及初始姿态

模型有 36 links、35 joints：**7 个机械臂转动关节 + 16 个手部转动关节 + 12 个固定连接**。手部 10 个独立输入、6 个被动联动，总独立维度 17，总状态坐标 23。

OpenArm `openarm_right_joint1`～`7` 的父子关系、原点、轴、零位和限位与验收模型逐元素一致。安装法兰、保留的 J8 安装件、手掌、tool_tcp 均保持原变换；没有增添 J8 电机或手部硬件接口。

| 手部 | 独立关节（前缀 omni_R_，后缀 _joint） | 被动联动 |
|---|---|---|
| 拇指 | thumb_roll、thumb_abad、thumb_mcp | thumb_pip=1.33×MCP；thumb_dip=1.3×MCP |
| 食指 | index_abad、index_pip | index_dip=1.097×PIP |
| 中指 | middle_pip | middle_dip=1.097×PIP；middle_abad 保持真正 fixed |
| 无名指 | ring_abad、ring_pip | ring_dip=1.097×PIP |
| 小指 | pinky_abad、pinky_pip | pinky_dip=1.097×PIP |

掌部、指尖、法兰和安装件固定连接没有被改成转动关节。全部关节的名称、父子 link、类型、轴、原点、限位、初始角和 mimic 详见 `reports/JOINT_DEFINITIONS.md` / `manifest/joint_manifest.json`。

收拢拇指依次为 Roll=0.6、ABAD=-0.5、MCP=0.45、PIP=0.5985、DIP=0.585 rad；四指 PIP=0.05、DIP=0.05485 rad。收拢不是完全握拳。`open_hand` 为官方零角度的伸展候选，不宣称最大外展；中间姿态是两者均值。初值是配置，不再烘焙成 fixed origin。

原始 MCP 上限 0.855211333×1.33 会略超 PIP 上限 1.134464014。URDF 保留来源限位，GUI 采用数学交集上限 **0.8529804615 rad**，并交付考虑 mimic 的有效位置/速度范围。没有任意扩限；GUI 仍不是避碰或真实电机保护器。

## 4. 验证结果及证据

| 验证 | 实际结果 | 证据 |
|---|---|---|
| ament 构建、URDF/Xacro 解析 | 描述包构建、check_urdf 通过；包内原始源重建字节相等 | build_final.log、check_urdf.log、assemble_model.py |
| 树/资源/轴/范围/mimic | 名称唯一、单连通树、资源可解析、23 个转动关节定义有效、全部 17 独立关节扰动确实改变对应 link | structural_validation.json、official_joints_and_coupled_limits.json |
| 收拢姿态对原验收模型 | 全部 36 link 矩阵最大元素差 `2.220446049250313e-16`；七轴和非手部关节定义完全保持 | structural_validation.json |
| 几何/惯量 | 所有 link 的视觉、碰撞、惯量字节语义保持（仅 URI 重定位）；源网格 SHA 不变 | structural_validation.json、model_manifest.json |
| MoveIt 有限碰撞检查 | 3 示例 + 21 过渡采样，范围全部有效；既有 SRDF 下无未豁免碰撞 | moveit_collision_final.json、collision_reproduction.txt |
| 更严格的无豁免对照 | 仍有原 31 对装配/包络交叠；从收拢到张开未增加接触对 | moveit_unmasked.json、collision_summary.json |
| RViz GUI 实际滑块 | MCP 从约0.44992668调到0，PIP/DIP随联动变化；画面和35条TF核对通过 | gui_before.json、gui_after.json、gui_controls.png、rviz_gui_slider_changed.png |
| RViz 三示例往返 | 23个状态值精确匹配，闭合→中间→张开→闭合；全景与局部截图已保存 | rviz_pose_demo_whole.json、rviz_pose_demo_closeup.json、rviz_examples_tf.json |
| Isaac 模型与运动学 | 24刚体、23转动关节、6 PhysX mimic；三姿态全部连杆FK最大元素差约`5.11e-7` | isaac_result.json |
| Isaac 默认初值 | 无运行器 reset，直接初始化资产为收拢状态，最大角误差`2.384e-8 rad` | isaac_final_asset_check.json |
| PhysX 实际运动 | 4800步×5ms=24秒；墙钟约29.15秒；目标驱动+实际积分，没有在动态循环重写位置/Xform | isaac_steps.json、isaac_demo_final.log |
| PhysX 误差与接触 | 最大跟踪误差0.00342853 rad；mimic残差≤2.873e-5 rad；末端目标角误差1.55e-7 rad；本次接触张量0 | isaac_summary.json |
| MuJoCo | 实际加载与三姿态渲染/FK通过，最大矩阵差3.613e-13；有腕部凸包接触，未积分 | mujoco_result.json、mujoco_*.png |
| 迁移验收 | TAR解压到新/tmp目录，校验、构建、RViz GUI启动和滑块运动、35条TF通过；原工程模型文件实际加载数0 | relocation_result.json、relocation_*.json、relocated_rviz_gui_*.png |

截图来自实际 RViz/Isaac/MuJoCo 渲染，不是合成图片。RViz截屏只裁去无关桌面区域，未修改模型画面；原桌面图留在本地 runtime，不打入可迁移包。Isaac几何相同但采用统一材质，颜色不作为几何一致性证据。

## 5. 平台支持边界

| 平台 | 文件生成 | 实际解析/加载 | 实际可视化 | 关节运动 | 物理积分 |
|---|---|---|---|---|---|
| ROS 2 Humble / RViz 2 | 是 | 是 | GUI通过 | GUI与状态示例通过 | 不适用 |
| Isaac Sim 6.0.1-rc.7 | 是 | 是 | GUI截图通过 | 全手示例与mimic通过 | 仅此次24秒零重力测试通过 |
| MuJoCo 3.4.0 | 是 | 是 | 实际离屏渲染通过 | 运动学状态变化通过 | 未通过，初始凸包接触即停止 |
| Gazebo | 未生成SDF | 未验证 | 未验证 | 未验证 | 未验证 |
| Isaac Lab RL | 未生成RL任务 | 未验证 | 未验证 | 未验证 | 未验证 |

Isaac Lab 3.0.0 在本机存在，但本轮使用 Isaac Sim API。不得将可加载 USD 描述为已完成强化学习环境。MuJoCo不能将它的凸包几何与PhysX的SDF接触结果直接等同。

## 6. 物理、碰撞和来源限制

1. 手部采用既有约447.1g归一化模型，质心/惯量未实测。全机器人惯量保留来源数值；没有虚构实物参数或宣称动力学已标定。固定基座、零重力、仿真驱动增益只是本次演示设置。
2. 原31组SRDF豁免保留，未新增。部分 base/tip link 有视觉但没有独立碰撞体，这是继承模型限制；不能把有限无新接触解释为完整手指包络认证，也不能把任意GUI姿态解释为无碰撞。
3. PhysX 保留SDF521/6及自碰撞；沿用相同源面数据，烘焙可能命中既有同几何缓存，不宣称全部重新烘焙或消除封孔/包络不确定性。原工具的每关节collision属性被PhysX articulation忽略会给警告，实际依靠 articulation自碰撞、明确的既有过滤对和接触张量。
4. MuJoCo mesh凸包在 `openarm_right_link5`/`link7` 出现约8.017mm接触，故不将MJCF列为可靠动力学或训练就绪模型。没有通过删除碰撞检查放行。
5. `closed_hand`与实物当前姿态未对照，SDK模型参数未经本轮物理测量。可动模型、可视化、仿真和真实手指控制是四种不同能力；本轮没有真实控制验证。
6. OpenArm Apache-2.0、OmniHand MulanPSL-2.0全文已附。派生安装网格没有独立再分发许可文本，OmniHand快照无Git commit。资产可在技术上独立迁移，不宣称具有无条件公开再分发权。

## 7. 修复与隔离

仅修改新的独立目录。修复了环境numpy/scipy混装（ROS检查使用系统Python）、SRDF机器人名字匹配、MuJoCo离屏尺寸、PhysX初始关节状态必须明确author、mimic有效限位，以及迁移入口工作目录。失败记录保留，没有删碰撞体或更改七轴定义使检查通过。

所有ROS/Isaac/MuJoCo运行入口通过现有AF_CAN seccomp屏障；迁移运行进程的Seccomp=2、socket系统调用中AF_CAN=0，直接证据为 `relocation_seccomp.json`、`socket_calls.txt` 和压缩系统调用日志。默认launch只含RSP/JSP/RViz；没有controller_manager、真实插件、SDK控制或旧J7 Runner。

91个注册原始输入文件的开始/结束SHA256全部相同，见 `manifest/source_hashes_before.json`、`source_hashes_after.json` 与 `reports/baseline_integrity.json`。核验范围为本次使用/复制的输入，不冒称对整台机器或全部无关历史文件做过逐字节审计。

## 8. 启动与交付

从任意位置解压，进入 `openarm_omnihand_portable_model`：

```bash
bash tools/build.sh
bash tools/display.sh
# 或精确手型示例：
bash tools/display.sh gui:=false
# 另一个终端：bash tools/set_pose.sh open_hand
```

Isaac：`ISAAC_SIM_PATH=/isaac-sim bash tools/isaac.sh --gui`。MuJoCo有限检查：`MUJOCO_GL=egl python3 tools/no_can.py python3 tools/mujoco_check.py`。其余源重建、碰撞工具、平台映射和依赖见根 README。

最终 ZIP/TAR.GZ、精确大小与SHA256见旁置最终报告和 `dist/archive_receipt.json`。包内 `manifest/asset_hashes.json` 验证模型与工具。实验结束后停止本轮GUI、ROS和Isaac进程，不自动开展真实硬件工作。
