# 官方增益 PPO 对照训练

本轮实机采用完整 MIT 控制，官方 V1.0 增益 Kp=[70,70,70,60,10,10,10]、Kd=[2.75,2.5,2,2,0.7,0.6,0.5]。本次仅替换仿真配置 physics.kp/physics.kd，与原策略进行对照；PPO 算法、奖励、观测/动作定义、A/B 目标、手姿、控制频率和原有随机化均保留。没有新增延迟、反馈量化或辨识摩擦模型，因此不能称为完整电机固件仿真，也不保证新策略直接满足实机要求。

`prepare_official.py` 从已有训练 run/task 中读取实际快照，生成新的任务目录。配置严格比较，除 Kp/Kd 外不得变化；A/B 不重新搜索，编译模型二进制原样复制，因为本工程 PD 在环境代码中计算，增益没有编入模型。新任务获得新的配置和任务哈希，旧运行目录绝不修改。原模型清单保存在 source_model.manifest.json，仅作源记录；新来源、差异和哈希记录在 official_variant.json。重新检查原 A/B 路径和2秒零动作保持，通过才设置新任务 preflight_passed。

`train_official.py` 调用同一个 train.py，从头初始化新的 PPO，不加载旧策略继续训练。默认继承旧配置，并保留旧 training_settings.json 中实际使用的 steps/envs/rollout，输出新的 outputs/ppo_official/run_日期目录。原正式任务通常为100万步、2个环境、rollout=1024；其他 PPO 配置包含学习率0.0003、batch=256、epochs=10、gamma=0.99、GAE=0.95、clip=0.2、entropy=0.005、网络[128,128]，以实际源快照为准。

旧模型和 PPO 文件保留。训练程序只在仿真中运行，不打开 CAN、不连接 ROS、不控制机械臂。real_robot/mit_servo.py 中官方增益是统一来源，训练准备程序只导入其常量，不启动任何硬件。

Ubuntu 操作：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
source .venv-ppo/bin/activate
python -m rl_ppo.prepare_official --source-run outputs/ppo/run_20261028_164711_538719
python -m rl_ppo.verify --task outputs/ppo_official/task/task.json
python -m rl_ppo.train_official --steps 2048 --envs 1 --rollout 128
python -m rl_ppo.train_official
```

第一条准备命令只执行一次，重复时拒绝覆盖。需要另一份任务请用新的 --output，并在 train_official 使用对应 --task。短训练显式覆盖 steps/envs/rollout，仅验证流程；正式训练不加这些选项，自动使用源正式运行设置，每次输出新 run 目录，不会覆盖短训练。

训练结束后，使用打印的**新 run 路径**评估：

```bash
python -m rl_ppo.evaluate --run outputs/ppo_official/run_实际日期目录 --episodes 20
python -m rl_ppo.evaluate --run outputs/ppo_official/run_实际日期目录 --episodes 20 --randomized
```

后续实机控制必须明确换成新 run，重新生成与新配置哈希匹配的启动计划和离线评估。旧的 real_robot/ppo_native_evaluation.json、旧任务的启动计划不能混用；此前 A 报告使用旧配置哈希，也不能直接冒充新任务的实机验收。先完成此次对照训练及仿真评估，再按新任务准备实机测试。
