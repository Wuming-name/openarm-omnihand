# OpenArm + OmniHand Ubuntu 22.04 / ROS 2 Humble 工程

本版本已补充 Gitee / Windows / Ubuntu 双机协作配置，完整步骤见 [Gitee双机协作](docs/Gitee双机协作.md)。Ubuntu 推荐项目位置为 `/home/lsk/ros2_projects/openarm_omnihand`；继续复用 `/opt/ros/humble`，不需要重新配置 ROS，也不需要将项目写入系统 ROS 安装目录。

此工程根据你提供的 `ubuntu_rl_env_20261025_193810.txt` 配置：Ubuntu 22.04.5、x86_64、i7-7700、约 16 GB 内存、约 6 GB NVIDIA 显存、470.256.02 驱动、系统 Python 3.10.12、`/opt/ros/humble`。

当前交付是模型工程与后续 RL 的准备基础：ROS 构建/展示入口、隔离的可选 MuJoCo 环境、模型完整性检查、关节输入映射和来源材料。尚未包含可训练的任务环境、策略或实机控制接口。

## 在 Ubuntu 中开始

将压缩包复制到 Ubuntu，解压后进入本工程根目录，直接运行：

```bash
bash project.sh check
bash project.sh build
bash project.sh display
```

无需手工 source，不要用 sudo 启动。入口会在子进程中加载 `/opt/ros/humble/setup.bash`，使用 `/usr/bin/python3`，清理继承的 ROS/Conda 路径，并且只构建本工程中的描述包。原有 ROS 1/catkin 工作区不会加入本次构建。默认 ROS domain 为 188。关闭窗口或 Ctrl-C 结束展示。

精确展示三个手型：

```bash
# 终端一：关闭 GUI 滑块，改用模型姿态播放器
bash project.sh display gui:=false

# 终端二：进入同一个工程目录后执行其中一个命令
bash project.sh pose open_hand
bash project.sh pose intermediate_hand
bash project.sh pose closed_hand
```

每次过渡约 8 秒。GUI 模式和播放器模式二选一，避免两个状态发布者。收拢手型不是完全握拳；三个示例中的机械臂角度保持相同，主要演示手部变化。它们是合成模型状态，不是实机反馈。

手部近景：

```bash
bash project.sh display gui:=false view:="$PWD/model/ros2_description/rviz/hand.rviz"
```

## 工程结构

| 路径 | 用途 |
|---|---|
| `project.sh` | 统一命令入口 |
| `model/ros2_description/` | ROS 主模型、网格与展示程序，唯一一份描述包源码 |
| `model/simulation/` | 保留 Isaac USD、MuJoCo MJCF/OBJ 和重建用 NPZ |
| `model/sources/`、`model/LICENSES/` | 原始来源、重建证据、许可 |
| `model/reports/`、`model/screenshots/` | 原 Linux 工程的历史验收证据，不是本台式机实测 |
| `config/desktop_profile.json` | 从你反馈报告提取的台式机配置和未知项 |
| `config/policy_contract.json` | 17 独立输入、23 坐标、单位、限位与联动定义 |
| `scripts/` | 工程检查、ROS 入口、CPU MuJoCo 检查、坐标映射 |
| `docs/` | 运行依赖、适配说明、后续 RL / sim-to-real 路线 |
| `manifest/` | 本工程哈希、源包身份及制作时验证结果 |
| `workspace/` | 在 Ubuntu 构建时生成的 build/install/log，不随包交付 |
| `.venv-mujoco/` | 可选 MuJoCo 虚拟环境，执行安装命令后生成 |
| `outputs/` | 本机新报告，与历史证据分开保存 |

## 可选 MuJoCo：先检查模型

这台电脑优先准备无渲染的 CPU 模型检查，不依赖 CUDA、PyTorch 或 Isaac。以下安装只写本工程的虚拟环境，需要网络：

```bash
bash project.sh setup-mujoco
bash project.sh mujoco-check
```

若创建虚拟环境时提示缺少 ensurepip/venv，先手工执行 `sudo apt install python3-venv`，再重试。无需安装完整 CUDA Toolkit。依赖固定为 MuJoCo 3.4.0 和 NumPy 1.26.4，以便和已有模型记录对照；尚未在你的 Ubuntu 机器上安装验证。

**原模型仍有腕部凸包交叠。** CPU 检查仅计算三姿态运动学和接触，不进行物理积分；发现接触时保存 `outputs/mujoco_cpu_check.json` 并返回退出码 3。这是在报告尚未解决的问题，不是通过删除碰撞体或豁免来隐藏问题。退出码 0 也仅代表此项有限检查通过，不代表可以训练。

## 其他命令

```bash
bash project.sh verify        # 与发布包基线严格比较，日常改代码后可报告差异
bash project.sh gpu-info      # 补全显卡完整型号与计算能力
bash project.sh contract      # 查看后续适配用的关节映射约定
/usr/bin/python3 -B -m unittest discover -s tests -v
```

`check` 校验工作副本的模型结构和引用，允许正常修改代码；`verify` 另按发布清单严格核对全部文件。日常 Git 更新后先运行 `check`，不应将正常代码改动当作压缩包损坏。`rl_training_ready=false` 在本阶段是预期结果。新命令不会改写 `model/reports`。旧 `model/tools` 保留用于追溯，部分旧工具会写历史目录或依赖原工作区，不作为本工程推荐入口。

USD 以三个至多 20 MiB 的二进制分块存入普通 Git，避免 Gitee 单文件限制。压缩包和新 clone 中不携带完整 USD；`check`、`build`、`display` 会自动校验并恢复该文件，拉取更新后也会刷新未被本地修改的 USD 缓存。单独恢复可在 Ubuntu 运行 `bash project.sh prepare-assets`，或在 Windows 运行 `python -B scripts/project.py prepare-assets`。完整 USD 被 Git 忽略；编辑 USD 后需运行 `python -B scripts/project.py pack-usd`，提交更新后的分块和元数据。恢复过程会拒绝覆盖未打包的本地 USD 修改。

收到包后，先发回 `outputs/desktop_check.json` 和 `build` / `display` 的结果；若安装可选 MuJoCo，再发回 `outputs/mujoco_cpu_check.json`。后续按 `docs/RL与实机路线.md` 继续。
