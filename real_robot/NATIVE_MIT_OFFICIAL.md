# 完整 MIT＋官方 V1.0 默认增益

当前 A 点已改为手掌位置/姿态/稳定性验收，不再要求每轴误差0.01rad。接入 PPO 的新入口、离线评估及操作步骤见 `PPO_A_TO_B_NATIVE_MIT.md`；本文件下文到 A 的控制方式保持一致，原策略并未自动获得新增益下的验证。

本说明替代 MOVE_TO_A_TRAINING_GAINS.md 中的电脑外环 PD 和 J5/J6 Kd 覆盖方案。`move_to_a.py` 与 ROS2 `bridge_node.py` 共用 `NativeMITServo`。旧增益覆盖命令不再支持，防止误把电脑外环参数用于电机内环。

官方来源（2026-10-29 核对）：
https://github.com/enactic/openarm_description/blob/main/assets/robot/openarm_v1.0/config/arm/control_gains.yaml

| 关节 | MIT Kp | MIT Kd |
|---|---:|---:|
| J1 | 70 | 2.75 |
| J2 | 70 | 2.5 |
| J3 | 70 | 2.0 |
| J4 | 60 | 2.0 |
| J5 | 10 | 0.7 |
| J6 | 10 | 0.6 |
| J7 | 10 | 0.5 |

控制职责：轨迹给出 q/dq；电脑根据当前实测状态和保存模型计算重力/偏置前馈；MIT 命令携带上述 Kp/Kd、q/dq 和模型前馈，由电机执行 PD。电脑只估算 PD 总力矩用于保护和记录，不把估算 PD 再加进发送的前馈。主机 PD 不再执行为输出，避免双重控制。模型坐标到电机坐标为 q_motor=offset+sign*q，dq_motor=sign*dq，tau_motor=sign*tau；Kp/Kd 保持非负。全部七轴先编码成功后才开始发送，避免末轴非法参数导致前六轴已发。

保护含状态/温度/新鲜度、关节限位、超速、参考/循环间隔、严重目标领先、路径接触及端点验收。MIT 内环 PD 不能靠截断前馈实现总力矩限幅，因此软件检查 abs(P)+abs(D)+abs(FF) 的预算不超过训练/机械峰值中较小者减 0.1Nm；反馈力矩超过峰值立即停止，估计或反馈力矩超过连续阈值 0.5秒停止。检查是基于当前样本的预算，不能保证两帧之间的电机输出硬限幅。电机电流/力矩硬限制尚未验证，TMAX 仍只是协议编码范围；不自动修改寄存器或 TIMEOUT。

轨迹及 A 验收规则保持。到 A 后持续保持，先承托机械臂再 Ctrl+C；退出尝试失能，无法保证送达，失能后可能因重力下落。必须有可靠支撑及独立停止措施。手部保持已有握拳，本程序不操作或持续监控手部。原始训练文件和 PPO 不改，旧策略还没有在新控制参数下验证或重新训练。本次只验证到 A；完整 ROS2 校准门槛保持，未开放自动 A→B 实机任务。

`move_to_a.py` 默认无 CAN 预览：

```bash
python -m real_robot.probe &&
python -m real_robot.plan_startup --run outputs/ppo/run_20261028_164711_538719 &&
python -m real_robot.move_to_a --run outputs/ppo/run_20261028_164711_538719
```

最后一条加 `--execute` 才控制硬件。启动时七轴须未使能，执行前两次核对实际位置与新计划。初始使能前的中性零力矩帧仅用于预加载；运行期为完整 MIT 五参数命令，不使用之前的主机力矩 PD。

报告 control_profile=openarm_v1_official_native_mit，host_pd_enabled=false；kp/kd/native_mit_kp/native_mit_kd 为实际电机原生增益，training_kp/training_kd 仅为训练基线记录。轨迹明确分开 feedforward_torque_nm、estimated_total_torque_nm 与 feedback_torque_nm，不能把估计总力矩称为实际命令前馈。

离线 startup_plan.json 中保留的 intended_gain_scale=0.2 是旧调试字段，当前完整 MIT 入口不读取它作为增益；该入口固定使用上表官方值。规划参考速度继续使用训练速度上限的 20%，与电机 Kp/Kd 比例无关。MIT 线编码会量化参数，解码值与表中浮点数有微小差别。

ROS2 mock 只用同一公式在 MuJoCo 计算理想 PD，无法证明电机内环响应稳定。ROS2 实际链路、原生 MIT 实物响应及固件硬限幅仍需分别验证。历史 hold_test/commission_hold/startup_joint_test 保留用于旧报告复现，不作为当前到 A 的入口。
