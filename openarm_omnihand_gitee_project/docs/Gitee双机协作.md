# Gitee 双机协作

## 已确认的 Ubuntu 环境

2026-10-26 的检查确认 `/opt/ros/humble` 正常，加载后 ros2、rclpy 和 RViz 均可定位。当前终端没有自动加载 ROS。家目录扫描发现的是 ROS 1 的 catkin 工作区，未在扫描范围内发现已构建的 ROS 2 工作区；`.bashrc` 中相关 ROS 1 source 行均已注释。Git 版本为 2.34.1。

本项目建议放在 `/home/lsk/ros2_projects/openarm_omnihand`。ROS 环境仍用 `/opt/ros/humble`，构建缓存位于项目自己的 `workspace/`。这不需要重新安装 ROS，也不会将本项目混入旧四足机器人的 catkin 工作区。

## 一次性：创建云端仓库

在 Gitee 网页创建一个空的私有仓库，例如 `openarm-omnihand`。不要勾选自动初始化 README、许可证或 gitignore，因为本工程已包含这些内容。复制仓库的 HTTPS 克隆地址。

本次未登录你的 Gitee，也未创建或推送任何云端内容。以下命令由你在各电脑执行。不要把密码或私人令牌写进仓库地址、代码或聊天；终端认证时按 Gitee 的提示输入。启用双因素认证的 HTTPS 操作可能需要以私人令牌代替密码。

## 一次性：Windows 笔记本首次推送

解压本次 Gitee 工程包，在含 `project.sh` 的项目根目录打开 PowerShell，然后执行：

```powershell
git --version
git init -b main
git config user.name (Read-Host '输入 Git 提交显示名')
git config user.email (Read-Host '输入 Git 提交邮箱')
git config pull.ff only
git add .
git status --short
git commit -m "Initialize OpenArm OmniHand Humble project"
$repoUrl = Read-Host '粘贴 Gitee 仓库的 HTTPS 克隆地址'
git remote add origin $repoUrl
git push -u origin main
```

如果 `git --version` 不存在，先安装 Windows Git，再重新打开终端。仓库地址形如 `https://gitee.com/你的账号/openarm-omnihand.git`；不要直接复制示例账号。这里只设置本仓库身份，不改变其他工程的全局身份设置。

## 一次性：Ubuntu 台式机克隆并运行

```bash
read -r -p '粘贴相同的 Gitee HTTPS 克隆地址: ' repo_url
mkdir -p ~/ros2_projects
cd ~/ros2_projects
git clone "$repo_url" openarm_omnihand
cd ~/ros2_projects/openarm_omnihand
git config pull.ff only
bash project.sh check
bash project.sh build
bash project.sh display
```

如果该目标文件夹已存在，不要覆盖或删除；请选择一个新的克隆目录，或先确认其中内容。新的 clone 不含 Windows 的构建缓存，第一次需要 build。之后不需要重装 ROS；代码、模型或配置更新后重新 build 即可。

## 日常：Windows 修改代码并推送

先在项目根目录执行：

```powershell
git status --short
git pull --ff-only
```

确认工作副本干净，再修改程序。修改后可以在 Windows 运行纯 Python 测试；它不运行 ROS 或 MuJoCo：

```powershell
python -B scripts/project.py prepare-assets
python -B -m unittest discover -s tests -v
git diff --stat
git add .
git commit -m "Describe this change"
git push
```

没有 Python 时仍可编辑和推送，由 Ubuntu 执行测试。Windows 笔记本不需要安装 ROS。不要把暂存的大量训练结果当作代码提交。

## 日常：Ubuntu 拉取并运行

先结束正在运行的展示/仿真，再执行：

```bash
cd ~/ros2_projects/openarm_omnihand
git status --short
git pull --ff-only
bash project.sh check
bash project.sh build
bash project.sh display
```

`git status --short` 如果显示未提交的代码修改，先保留并提交这些修改再切换到另一台电脑工作。`--ff-only` 遇到分叉时会停止，不会替你重写或丢弃提交。不要用 force push、reset 或删除目录来绕过提示。

台式机上也可以修改代码；如需提交，先在该仓库设置 user.name/user.email，然后 commit/push，回到笔记本先 pull。两个电脑不会自动同步，只有成功 push/pull 后才共享提交；不需要两台电脑同时开机。

## 大模型和 Git 规则

完整 USD 为 54,153,384 字节，超过 Gitee 文档列出的免费版单文件 50 MB 限制，所以普通 Git 只跟踪 `assets/usd_parts/` 的三个分块及 `config/large_assets.json`。不依赖额外的 Git LFS 服务或插件；分块按原文件字节切割，恢复时同时核验分块与整文件 SHA256。

新 clone 的 `check`、`build` 会自动恢复 USD。拉取新分块后，如果本地 USD 没有被修改，会更新缓存；如果存在未打包的本地修改，会停止并保留修改。确实编辑 USD 后，在改动所在电脑执行：

```powershell
python -B scripts/project.py pack-usd
git add assets/usd_parts config/large_assets.json
git commit -m "Update USD asset"
git push
```

Ubuntu 也可以使用 `bash project.sh pack-usd`。模型其他文件正常由 Git 跟踪。不要手动编辑分块。原发布清单仍供追溯；模型或代码修改后 strict verify 与原发布不同是预期结果，日常使用 check 和测试。

`.gitattributes` 固定可编辑源码的 LF 换行，并保留冻结模型的原字节；`.gitignore` 排除构建缓存、虚拟环境、运行输出、训练日志、权重、压缩包和本地密钥环境文件。配置中无需写死 Windows 或 Ubuntu 的项目绝对路径。

参考：

- Gitee 仓库体积与单文件限制：https://gitee.com/help/articles/4232_
- Gitee 双因素认证与令牌：https://blog.gitee.com/2026/07/14/gitee-%E5%8F%8C%E5%9B%A0%E7%B4%A0%E8%AE%A4%E8%AF%81%EF%BC%9A%E4%B8%BA%E4%BB%A3%E7%A0%81%E8%B5%84%E4%BA%A7%E5%8A%A0%E4%B8%8A%E7%AC%AC%E4%BA%8C%E6%8A%8A%E9%94%81/

RL 与实机所需的模型修复和任务定义仍见 `RL与实机路线.md`，Git 协作配置不会使尚未验证的动力学变成训练就绪模型。
