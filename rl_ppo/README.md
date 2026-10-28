# OpenArm V1 + 固定握拳 OmniHand：PPO 掌心到达训练

这是独立的 **仿真训练模块**，控制机械臂七轴，手部冻结成固定负载。没有 ROS/CAN 导入、硬件连接、自动上电或实机部署入口。训练出的权重不能直接发送给电机。已有 `project.sh`、ROS 2 环境和手部测试工具保持原用途。

详细逐文件、逐流程说明见 [代码与训练说明.md](代码与训练说明.md)。

## Ubuntu 安装：单独虚拟环境

在 Git 工程根目录执行。不要在这个虚拟环境里编译 ROS，也不要覆盖系统 Python：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
git pull --ff-only
python3 -m venv .venv-ppo
source .venv-ppo/bin/activate
python -m pip install --upgrade pip
python -m pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip install -r rl_ppo/requirements.txt
```

若提示没有 venv 模块，安装 Ubuntu 对应的 `python3-venv` 后再创建。若下载失败，应先修复 pip 网络/代理；不需要升级 CUDA 或改 ROS。trimesh 及可选 CoACD 仅重做碰撞部件拆分时需要，正常训练使用已附带的部件，不必安装。

## 第一步：生成任务

```bash
python -m rl_ppo.prepare
```

生成目录为 `outputs/ppo/task/`。会冻结手部、启用重力、应用腕部碰撞部件拆分、搜索相距 20～30 cm 的 A/B、检查 40 个连接姿态，并验证 2 秒静态保持。失败就停止，不跳过碰撞或动力学检查。运行时可能需要几十秒，取决于机器。

不覆盖已有任务。修改配置后使用新目录：

```bash
python -m rl_ppo.prepare --output outputs/ppo/task_v2
```

使用新任务目录后，后续命令的 `--task` 也要改为 `outputs/ppo/task_v2/task.json`，包括 `python -m rl_ppo.train --task outputs/ppo/task_v2/task.json`；不指定时训练仍读取默认 task 目录。

## 第二步：检查环境与参考轨迹

```bash
python -m rl_ppo.verify --task outputs/ppo/task/task.json
python -m rl_ppo.inspect_task --task outputs/ppo/task/task.json --baseline
```

`--baseline` 是传统关节参考控制在仿真中的诊断，不是 PPO，也不是实机运动。先确认这个诊断成功，再长时间训练。

有 Ubuntu 图形桌面时，可以分别查看 A、B 和参考动作：

```bash
python -m rl_ppo.inspect_task --task outputs/ppo/task/task.json --pose A --render
python -m rl_ppo.inspect_task --task outputs/ppo/task/task.json --pose B --render
python -m rl_ppo.inspect_task --task outputs/ppo/task/task.json --baseline --render
```

关闭窗口结束姿态预览。绿色点是掌心 TCP，红色点是目标。默认无渲染训练无需显示器或 GPU。

## 第三步：先运行短程训练

```bash
python -m rl_ppo.train --steps 2048 --envs 1 --rollout 128
```

这只检查训练、保存流程，不期待学会任务。终端会打印 `RUN: ...`，记下该目录。

正式基线训练：

```bash
python -m rl_ppo.train
```

默认 CPU、2 个进程、100 万环境步、策略周期 20 ms、两层 128 单元 MLP。实际完成步数按完整 rollout 向上取整。每个环境都有自己的模型和随机数状态。完成一个训练任务不代表达到目标成功率；检查日志和独立评估结果后决定是否调整奖励、模型或训练步数。

## 第四步：独立评估

将下面 `run_实际目录名` 替换为训练输出目录：

```bash
python -m rl_ppo.evaluate --run outputs/ppo/run_实际目录名 --episodes 20
python -m rl_ppo.evaluate --run outputs/ppo/run_实际目录名 --episodes 20 --randomized
```

第一条使用名义动力学，第二条加入质量、增益及测量噪声变化。两者都从 A 附近出发、前往完整 B，不使用训练早期较近的课程目标。保存成功率、终点误差、终止原因和首个回合轨迹。

查看策略动作：

```bash
python -m rl_ppo.evaluate --run outputs/ppo/run_实际目录名 --episodes 1 --render
```

评估历史最佳模型：

```bash
python -m rl_ppo.evaluate --run outputs/ppo/run_实际目录名 --model best/best_model.zip --episodes 20
```

## Windows 与 Ubuntu Git 同步

Windows 仅编辑与离线验证即可，正式训练建议 Ubuntu：

```cmd
cd /d "D:\Projects\openarm_omnihand_gitee_project"
git add rl_ppo
git commit -m "Add frozen-hand OpenArm PPO reaching training"
git push
```

Ubuntu `git pull --ff-only` 后按上述步骤运行。`outputs/` 和 `.venv-ppo/` 已被工程现有 `.gitignore` 排除。权重、训练日志和生成的模型不上传普通 Git。

**当前边界：**掌心偏移、固定拳形、MIT 增益、力矩上限和实际电机动态尚未完成实机标定。仿真中成功后仍需独立完成 ROS 2 状态机、机械臂至 A 的轨迹、策略接管、手部保持和异常停机逻辑。
