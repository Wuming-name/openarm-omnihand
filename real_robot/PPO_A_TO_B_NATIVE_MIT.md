# 接受当前 A 点误差后执行 PPO 到 B

## 官方增益重训后的新策略

新训练 run 为 `outputs/ppo_official/run_20261029_213104_305061`，后续命令必须明确使用它。标准 A 附近的20次标称及20次随机化评估已由用户完成；下一步仍需从接受的实际 A 姿态做一次匹配新策略的离线评估。

旧 A 报告 `outputs/real_robot/20261029_201652_989760_move_to_A.json` 不改写。程序只在核对 official_variant.json 的来源链、模型及 A/B几何不变、新配置仅替换 Kp/Kd、原实物报告确实使用同一官方 MIT 增益后，复用其中的实测位置，并在新评估报告中保存 accepted_A_lineage。不是把旧策略评估报告改成新策略报告，也不认定新策略已实机成功。

```bash
python -m real_robot.evaluate_policy \
  --run outputs/ppo_official/run_20261029_213104_305061 \
  --start-report outputs/real_robot/20261029_201652_989760_move_to_A.json \
  --output outputs/real_robot/ppo_official_A_evaluation.json
```

仅当 ALL_PASSED=True 后执行：

```bash
python -m real_robot.probe &&
python -m real_robot.plan_startup --run outputs/ppo_official/run_20261029_213104_305061 &&
python -m real_robot.run_policy \
  --run outputs/ppo_official/run_20261029_213104_305061 \
  --evaluation-report outputs/real_robot/ppo_official_A_evaluation.json --execute
```

以下较早命令中的旧 run 仅为历史示例，以本节新 run 为准。实机仍使用完整 MIT 和官方默认增益，到 B 后持续保持，先承托再 Ctrl+C。模型计算误差不是外部实测精度；这轮仍是 SocketCAN 测试入口，正式 ROS2 自动任务尚未验证。

此轮使用与实物到 A 测试相同的 SocketCAN 调试入口和完整 MIT 控制器，不是 ROS2 自动部署已完成的声明。ROS2 bridge 继续共用相同 MIT 控制实现，但正式 ROS2 校准门槛保留。训练文件、策略权重、模型和哈希不修改。

`move_to_a.py` 原有到 A 保持模式保留，A 验收改为手掌距离不超过 15mm、姿态误差不超过 0.10rad、七轴速度不超过 0.05rad/s并稳定 0.5秒，不再以七轴逐个 0.01rad 误差作为准入。用户接受的读数手掌误差约 5.396mm，关节偏差约 0.024rad，也能通过这项模型验收。位置和姿态仍是编码器＋模型计算，不是外部测量。

`policy_mission.py` 管理 HOLD_INITIAL → shoulder_retreat → wrist_retreat → HOLD_A（中间 to_A 轨迹）→ POLICY → HOLD_B。A→PPO 时使用实时 q/dq 生成原始 37维观测，保留最后一个 A 目标作为策略目标积分器初值，速度和上一动作归零，避免目标突然从 q_A 跳到实测 q。PPO 首次增量仍受原来的加速度、速度和目标领先限制。策略在独立模型进程中推理，不占用 500Hz CAN 控制循环。

`run_policy.py` 是新实物任务入口，共用现有到 A 主循环。七轴始终使用官方 MIT Kp=[70,70,70,60,10,10,10]、Kd=[2.75,2.5,2,2,0.7,0.6,0.5]；电脑只发送模型前馈，不叠加主机 PD。策略不控制手指，不改变已经握拳的手。程序没有手部持续监测，操作者仍须确认手保持目标姿态。

到 B 沿用训练条件：手掌距离<20mm、姿态误差<0.25rad、手掌速度<0.03m/s，稳定0.5秒。策略阶段超过训练的12秒仍未达标则失败。达标后停止策略更新，冻结最后关节目标并持续保持 B；短暂反馈波动允许重新稳定，连续0.5秒不满足终点条件则报错。屏幕输出 POLICY、HOLD_B、B_REACHED，正常或异常退出均尝试失能。

`evaluate_policy.py` 在官方增益下进行纯离线 PPO 评估，从已接受的实际 A 关节姿态起步，保留 A 目标，分别做20次标称和20次随机化评估。随机化包含原配置质量/增益/阻尼/观测噪声，另对起点各轴扰动±0.015rad。报告明确是理想 MuJoCo 执行器公式，不模拟完整固件、电流环、真实延迟或硬件硬限幅；每个50Hz策略采样处检查1rad/s部署速度限值，不能覆盖采样间的全部瞬态。它不是实机安全认证。

`run_policy --execute` 要求上述离线报告两组各至少20次全部成功，报告必须匹配当前策略文件、任务、模型、配置哈希及官方增益。旧训练增益的评价报告不会被接受。若离线失败，先分析报告或重新训练，不能跳过检查。默认无 --execute 不开 CAN，preview_only=true 仅表示启动路径预览，不表示 PPO 成功。

Ubuntu 先运行（不移动机械臂）：

```bash
python -m real_robot.evaluate_policy \
  --run outputs/ppo/run_20261028_164711_538719 \
  --start-report outputs/real_robot/20261029_201652_989760_move_to_A.json
```

仅当 ALL_PASSED=True 且退出码0，再执行：

```bash
python -m real_robot.probe &&
python -m real_robot.plan_startup --run outputs/ppo/run_20261028_164711_538719 &&
python -m real_robot.run_policy --run outputs/ppo/run_20261028_164711_538719 --execute
```

需要可靠且不阻碍运动的支撑和独立停止措施。B点保持时先承托机械臂再 Ctrl+C，失能后可能下落；故障也可能中途失能。保留全部通信、温度、硬限位、超速、力矩预算/反馈、参考和循环新鲜度、碰撞/工作空间保护。电机固件硬力矩限制与 TIMEOUT 行为仍未验证，不写寄存器、不伪造已验证状态。不同时运行其他机械臂控制器。
