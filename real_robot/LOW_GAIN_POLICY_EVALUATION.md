# 20%增益实机保持通过后的策略评估

用户实机报告20261028_214914_670360：20%软件PD增益、固定初始目标、有支撑条件下完成1500周期/3秒，平均499.9787Hz、最大循环间隔2.249ms，每轴观测到1482～1484次反馈更新，最大位置偏差0.0003815rad（约0.022°），未触发保护。该结果仅验收本次固定姿态条件；碰撞模型未通过运动验收，未运行PPO。退出发送了失能，但报告不证明失能投递。

现有策略训练使用完整增益。不能将20%增益保持成功解释为策略在20%增益下仍能完成A→B，因此先离线评估同一权重：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
source .venv-ppo/bin/activate
run_dir=outputs/ppo/run_20261028_164711_538719

python -m rl_ppo.evaluate --run "$run_dir" \
  --gain-scale 0.2 --episodes 20 --seed 3000

python -m rl_ppo.evaluate --run "$run_dir" \
  --gain-scale 0.2 --episodes 20 --seed 4000 --randomized
```

两条命令均不打开CAN、不使能电机、不控制手部。评估器先校验原任务/模型指纹，再仅在内存中将base_kp、base_kd乘以0.2；每次reset之后保留该基准，随机化围绕缩放后的增益执行。前馈、动作/观测、目标A/B、限幅和模型不改写。原task/config/model/权重都不修改。默认gain_scale=1保持原有评估行为。

报告文件名包含`evaluation_nominal_gain_0.2_...`或`evaluation_randomized_gain_0.2_...`，明确记录gain_scale、training_gains_unchanged=false、有效名义Kp/Kd及原模型/配置SHA，避免与原增益评估混淆。逐回合输出失败原因和最终误差。

先以20回合全部成功作为下一阶段的筛选条件；仍不代表实机运动安全或动力学已经标定。若有失败，保留结果，不直接接实机，也不要通过放宽阈值或偷偷恢复完整增益绕过，应重新评估增益选择或训练分布。当前模拟PD使用理想关节状态，真实驱动/传感器/延迟与负载误差未全部标定，仿真通过不能证明实际J6问题根因已经消除。初始碰撞几何和到A路径还需独立验证。

本地39项测试通过，新增检查确保20%基准在reset后正确使用且原配置不变。使用本地短训权重执行过一次评估以验证命令和报告流程；该短训权重没有完成任务训练，不能代替用户正式权重的成功率评估。
