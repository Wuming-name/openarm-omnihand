# OpenArm V1 + OmniHand：ROS 2 / 达妙 MIT / PPO 部署程序

对应流程：七轴保持当前姿态 → OmniHand 六轴缓慢握合、四轴保持 → 检查手部姿态 → 传统最小加加速度轨迹至 A → PPO 控制掌心至 B → 保持 B。全程监测手部使能与位置偏差。

软件默认使用模拟后端，不打开 CAN。实机代码已实现，但作者环境是 Windows，没有接入你的 ROS 2/CAN/电机，因此这里不声称已完成实机验收。首次请运行下面的 **只读 probe** 和 **无硬件 dry_run**，再在 Ubuntu 验证 ROS 2 模拟节点。

## 1. 官方参数：分清三个范围

已核对官方 `enactic/openarm_can` 提交 `bfd4e52194aefe38389fa6e670be3738136facb9`。

| 关节 | 实物型号 | 协议类型 | MIT 编码 P/V/T 对称量程 | 额定 / 峰值力矩 |
|---|---|---|---|---|
| J1、J2 | DM-J8009P-2EC | DM8009 | ±12.5 rad / ±45 rad/s / ±54 Nm | 20 / 40 Nm |
| J3 | DM-J4340P-2EC | DM4340 | ±12.5 rad / ±10 rad/s / ±28 Nm | 9 / 27 Nm |
| J4 | DM-J4340-2EC | DM4340 | 同上 | 9 / 27 Nm |
| J5～J7 | DM-J4310-2EC V1.1 | DM4310 | ±12.5 rad / ±30 rad/s / ±10 Nm | 3 / 7 Nm |

MIT 编码量程不等于机械允许速度、关节角范围或持续力矩。ID 默认使用 J1～J7 的 TX=0x01～0x07、RX=0x11～0x17，但会读取实际寄存器核对，不能只凭默认值认定实物已经设置正确。

官方 MIT 的 Kp 编码范围为 0～500，Kd 为 0～5。现有策略训练时 Kd 有 8、10、12，直接塞入报文或截到 5 会改变闭环。程序使用官方支持的 **Kp=Kd=0、前馈力矩模式**，主机每 2 ms 计算：

```text
tau = Kp_train * (q_cmd - q) + Kd_train * (v_cmd - qdot) + bias_feedforward
tau = clip(tau, -min(training_limit, mechanical_peak), +min(...))
MIT(q_des=0, v_des=0, kp=0, kd=0, t_ff=sign*tau)
```

报文 q_des 为零不会产生归零动作，因为报文 Kp=0。位置闭环和阻尼在主机上计算，电机内部执行电流/力矩控制。训练增益仍是该训练任务的增益，**不是厂家针对握拳负载标定的通用参数**。官方协议量程已核对，不代表质量、零点或动力学模型已被实机确认。

此方案依赖可靠的 500 Hz 主机控制。Python/rclpy 是软实时程序，不能保证硬实时；发生严重调度延迟会锁存故障。不要通过放宽超时掩盖 Linux 调度、CAN 或驱动问题。

## 2. 先做七轴只读检查

Windows 提交新增目录后，Ubuntu 项目根目录执行：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
git pull --ff-only
python3 -m real_robot.probe
```

probe 不依赖 ROS、PyTorch 或 MuJoCo，只使用系统 Python 标准库。仅发送官方参数读取 `0x33` 与状态刷新 `0xCC`；不发送使能、MIT 力矩、归零、写参数或存储指令。报告保存 `outputs/real_robot/*_probe.json`。

读取每轴 MST_ID、ESC_ID、CTRL_MODE、TIMEOUT、PMAX、VMAX、TMAX 和状态/温度。`official_protocol_match: true` 仅表示这些协议字段与本配置匹配，不表示标定、使能或实机策略已经通过。

程序只读检查 can0 必须已 UP、FD、1M/5M、ERROR-ACTIVE。若接口 DOWN，请先停止原机械臂控制程序，确认机械臂有支撑、允许停止原通信，再仅配置机械臂通道：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 sample-point 0.75 \
    dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on restart-ms 0
sudo ip link set can0 up
python3 -m real_robot.probe
```

2026-10-28 用户提供的 Ubuntu 报告 `20261028_194206_385603_probe.json` 显示：采用上面的 can0 时序后，J1～J7 的 ID、MIT 模式、P/V/T 量程全部读取并匹配，查询前后保持 ERROR-ACTIVE、tx/rx 错误计数为 0，错误警告/被动累计计数未增加。这仅是一次只读检查；不是 500 Hz 控制验收。七轴状态均为 0（未使能），TIMEOUT 原值均为 0，通信丢失保护尚未核实。不得把 0 填成“已验证的有限超时”，也不自动改写电机寄存器。

这不改变电机内部参数，也不配置手部 can1。只读查询并不接管原控制器；不要同时运行两套力矩发送程序。

如果实物寄存器量程不同，程序会拒绝控制，而不是使用错误量程解码。先保留报告核对固件和实际设置，不自动刷写成“默认值”。

## 3. 不连接硬件验证完整流程

使用你已经训练好的目录，在训练虚拟环境中运行：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
source .venv-ppo/bin/activate
run_dir=outputs/ppo/run_20261028_164711_538719
python -m real_robot.self_check --run "$run_dir"
python -m real_robot.dry_run --run "$run_dir"
```

self_check 比较部署端 37 维观测与 500 Hz 软件 PD 是否与原训练环境一致。dry_run 则在 MuJoCo 中执行包括至 A 的启动轨迹、策略和 B 保持在内的完整状态机，不用 CAN、ROS 或真实手部。

可视化全流程：

```bash
python -m real_robot.dry_run --run "$run_dir" --render
```

诊断状态机与控制器时还可使用 `--baseline`，由传统关节参考代替 PPO；此结果不计为策略验证。报告明确记录 baseline 标志。

## 4. ROS 2 模拟联调：两个终端

使用 Ubuntu 系统 Python 3.10 创建的现有 `.venv-ppo`。先 source ROS 再启用 venv，rclpy 由 ROS 的 PYTHONPATH 提供，不要 pip install rclpy：

每个终端都先执行：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
source /opt/ros/humble/setup.bash
source .venv-ppo/bin/activate
export ROS_DOMAIN_ID=188
python -c "import rclpy, sensor_msgs, std_srvs, torch, mujoco; print('ROS_PPO_IMPORT_PASS')"
```

终端 1，底层模拟 servo：

```bash
python -m real_robot.bridge_node \
    --run outputs/ppo/run_20261028_164711_538719
```

终端 2，上层任务（模拟手部）：

```bash
python -m real_robot.mission_node \
    --run outputs/ppo/run_20261028_164711_538719
```

默认两个后端都是 mock，不打开 CAN。完整到达后显示 DONE，但继续发布保持指令；退出任务节点会请求底层锁存当前位置保持。这里只验证 ROS 流程，仍不能证明真实 CAN 实时性能。

## 5. 实机前需要填写的真实信息

配置模板：`real_robot/config/hardware.json`。建议复制为 `real_robot/config/hardware.local.json`，本机副本已被 Git 忽略。禁止为了绕过拦截而随意把校准字段改为 true。

| 字段 | 需要的证据 |
|---|---|
| motors 的 tx/rx/protocol | probe 寄存器与实际 J1～J7 电机对应关系 |
| motors 的 sign/offset_rad | 实物关节角与训练模型相对应；`q_model=sign*(q_motor-offset)` |
| joint_mapping_and_zero_verified | 七轴方向、单位、软件零点偏移和限位已核实 |
| hand_pose_and_tcp_match_training | 实际六轴握合、四轴保持与训练的固定负载/TCP 在已确认容差内一致 |
| hardware_stop_and_support_verified | 断电、通信丢失时机械臂有物理支撑和可用硬件停止措施 |
| motor_timeout_raw_verified | 电机 TIMEOUT 寄存器的实际数值，可为一个值或七轴数组 |
| motor_timeout_seconds_verified | 对应的真实超时时间，要求 20～200 ms，可为一个值或七轴数组 |

官方寄存器表列出了 TIMEOUT 原始 uint32，但未在所核对页面明确给出单位，因此程序不猜测“原值就是毫秒”。先根据对应固件工具/文档或实际测量确认换算。程序只校验，不修改寄存器、不改变控制模式、不写 Flash。

实际手部四个轴此前未明显跟随，而训练使用 SDK 候选拳形；这一差异必须核对。不能仅以仿真 200/200 成功率把该校准项标成已完成。

## 6. 实机入口：校准和模拟联调通过后

保持两终端同一 ROS_DOMAIN_ID，停止旧驱动，不能同时发送 MIT 指令。模板中的 calibration 未核实字段会明确拦截以下命令；这是当前实物信息未完成的检查，不是已经验收通过的配置。

终端 1：

```bash
python -m real_robot.bridge_node \
    --run outputs/ppo/run_20261028_164711_538719 \
    --config real_robot/config/hardware.local.json \
    --backend socketcan --execute
```

默认要求电机已使能。如果已经确认七轴静止、有支撑且需要软件初次使能，可额外添加 `--enable-motors`；只允许一次初始使能，不会清故障或自动反复使能。

终端 2：

```bash
python -m real_robot.mission_node \
    --run outputs/ppo/run_20261028_164711_538719 \
    --config real_robot/config/hardware.local.json \
    --hand canfd --execute
```

禁止混合真实机械臂与模拟手部。首次目标必须保持静止的当前姿态，手部完成握合后才去 A。去 A 的路径从当前状态计算最小加加速度轨迹，逐点检查自碰撞；单轴离 A 超过 2 rad 会拒绝这条简化启动路径。启动段允许模型硬限位以内的姿态，PPO 段恢复训练软限位与工作范围。到 A 后要求关节误差小于 0.01 rad、速度小于 0.03 rad/s，再保持原目标无位置突跳接管。

没有实现任意起点的避障规划，80 点路径检查也不等于连续碰撞检测。需要更复杂的启动路径时应使用已验证的轨迹规划器，不要扩大阈值绕过路径拒绝。

## 7. 停止与故障行为

请求停止任务、锁存当前位置保持（需要底层与反馈仍然正常）：

```bash
ros2 service call /openarm_rl/stop std_srvs/srv/Trigger '{}'
```

锁存后不接受新的运动目标，需停机检查后重启节点。该服务和 Ctrl+C 都不是机械急停。

- 上层指令超过 100 ms 未更新、底层循环间隔超过 20 ms：反馈仍新鲜时锁存当前位置，继续受限软件 PD 保持。
- 七轴反馈超过 50 ms、异常状态码、越限/超速、温度超过应用阈值：停止继续发送控制。失联电机无法靠软件保持。
- 总力矩限制不超过训练上限和机械峰值；超过额定力矩持续 0.5 秒也停止控制。0.5 秒和 70℃ 是本程序的保守应用阈值，不是厂家承诺的过载时长。
- 通信完全丢失或底层节点退出后，只能依赖已经核实的电机超时保护。达妙 QDD 臂失能后不能靠程序保证支撑，可能在重力下下落；因此不要在无物理支撑时把杀进程/断通信当作安全停止。

锁文件只防止本程序重复打开 can0，不能阻止其他厂商程序发送。必须主动停止原控制程序。

## 8. 各文件职责

| 文件 | 作用 |
|---|---|
| protocol.py | 官方 MIT 位编码、状态/参数解码、FD 帧与本机回显排除，无归零/写参数接口 |
| settings.py / config/hardware.json | 通道、电机映射、机械限幅、看门狗及真实校准状态 |
| can_bus.py | Linux SocketCAN、参数读取核验、初次使能和总力矩发送 |
| probe.py | 七轴只读检查与 JSON 报告 |
| control.py | 训练快照加载、37 维输入、目标积分、至 A 轨迹、完整任务状态机 |
| servo.py | 500 Hz 软件 PD、力矩裁剪、持续力矩与命令看门狗 |
| bridge_node.py | 底层 ROS 2 节点，真实或模拟后端 |
| hand_worker.py | 独立 can1 线程，调用已验证的六轴握合工具并持续监测 |
| mission_node.py | 上层 50 Hz PPO 节点及手部/状态监测 |
| dry_run.py | 无 ROS/CAN 的整流程仿真，可显示动作 |
| self_check.py | 部署输入和控制数学与训练实现一致性检查 |
| tests/test_protocol_and_safety.py | 编码向量、范围、回显、限幅、看门狗与只读检查测试 |
| official_sources.json | 官方仓库锁定提交及开发时参考文件 SHA256 |
| VALIDATION.json | 本次真正执行的测试与未验收范围 |

ROS 2 接口：`/openarm_rl/joint_states` 为 JointState 实际状态；`/openarm_rl/joint_command` 为 JointState 格式的目标，其中 position=q_cmd、velocity=v_cmd、effort=名义动力学前馈；两个 status 话题为 JSON String。指令检查时间戳、关节顺序、唯一发布者、量程与领先量，拒绝过期和乱序指令。模型与配置哈希必须与策略快照一致。

## 9. 离线测试与 Git

```bash
python -m unittest discover -s real_robot/tests -v
```

在 Windows CMD 提交源文件：

```cmd
cd /d "D:\Projects\openarm_omnihand_gitee_project"
git add real_robot
git commit -m "Add ROS2 Damiao MIT PPO deployment and read-only checks"
git push
```

本模块不修改已训练 run、训练增益或原模型。实机标定不匹配时应修正训练环境并重新验证/训练，而不是悄悄修改快照以绕过哈希检查。

## 10. 官方资料

- [OpenArm CAN 官方参数](https://github.com/enactic/openarm_can/blob/bfd4e52194aefe38389fa6e670be3738136facb9/include/openarm/damiao_motor/dm_motor_constants.hpp)
- [OpenArm CAN 官方 MIT 编解码](https://github.com/enactic/openarm_can/blob/bfd4e52194aefe38389fa6e670be3738136facb9/src/openarm/damiao_motor/dm_motor_control.cpp)
- [OpenArm 官方 ROS 2 硬件接口](https://github.com/enactic/openarm_ros2/blob/8f23aeabb372c256f7807b9a74d48e035fc68386/openarm_hardware/src/openarm_simple_hardware.cpp)
- [达妙 MIT、反馈帧与寄存器说明](https://damiao.enactic.ai/en/products/hardware/dm-j4340p-2ec-v1.0/)
- [OpenArm 电机额定与峰值规格](https://github.com/enactic/openarm/blob/46cfc7c93ba271f2ff08823ab43536ee76184d32/website/docs/hardware/openarm-2.0/motor.mdx)

协议按这些官方来源独立实现；额定/峰值按用户已说明的相同电机型号选用，不能仅凭“OpenArm 版本”套用其他电机的值。
