# 右手 OmniHand O10 慢速握拳测试

适用于本次实物已验证的配置：物理 CAN2 / can1 / ID 0x09 / 软件 1.1.9 / 硬件 3.0.0。只运动手部，机械臂 CAN1/can0 不打开。无需安装智元 SDK 或其他 Python 库，使用已有 SocketCAN 和 Python 3.10 标准库。先确保 can1 沿用通信测试的 UP、FD、1M/5M 配置。

## 执行

手掌保持空载、手指周围留出活动空间，停止其他手部控制程序。位置模式不限制握力，不要让手握物体、夹住线缆或接触人的手。

Ubuntu 先预览（不会打开 CAN socket）：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
python3 -B tools/omnihand_can_test/fist_test.py --channel can1
```

执行一次缓慢握拳，结束后保持目标，不自动反复开合：

```bash
python3 -B tools/omnihand_can_test/fist_test.py --channel can1 --execute
```

需要张开时，使用同一官方右手 PAPER 预设缓慢张开：

```bash
python3 -B tools/omnihand_can_test/fist_test.py --channel can1 --gesture open --execute
```

`Ctrl+C` 停止发送后续目标，但不是电机急停：程序不会自动失能、回零或反向张开，设备仍可能继续追踪最后目标并保持力矩。发现夹碰、异响或异常发热时使用硬件断电措施。

## 实现与边界

- 首先查询型号、版本、使能状态和当前位置；要求已使能，不发送使能、失能、校准、清错或修改设备参数命令。
- 单轴位置命令 `0x06`，轴 1～10，每轴都要求同设备同轴位置应答；无应答立即停止后续目标。
- 从当前姿态插值到官方目标，单步每轴最多 24 计数，步骤频率不超过 10 Hz，即目标变化上限 240 计数/秒；完整握拳通常约 17 秒或更长。它限制的是发送目标的变化，不是电机速度或握力的硬限制。
- 每步读取 `0x0D` 故障码，非零或查询超时即中止。位置跟踪差超过 512 计数，或超过 128 计数持续 1 秒，也中止。软件检查不代替硬件急停，也不保证避免所有碰撞。
- 已观测到反馈 4097、4098。启动允许反馈 0～4112，只将规划起点截到 4096，原反馈保留；实际命令始终在 0～4096。16 计数余量是本工具的工程容差，不是厂商标定结论。
- 完成后重新读取十轴位置，与目标最大差不超过 128 计数才标记 `position_test_passed: true`。此标记表示位置目标检查通过，是否形成期望拳形还需目视确认。
- 动作期间不循环重试写指令，不改变 CAN 参数，不扫描设备，不打开 can0。运行报告保存到工程 `outputs/omnihand_can_test/*_motion.json`，已有 gitignore 排除。

## 预设来源

`right_hand_gestures.json` 由官方 SDK 1.1.10 的纯运动学求解器离线生成：`OmniHand2025Solver(False)`，调用 `OMNIHAND_2025_GESTURE_FIST1` 和 `OMNIHAND_2025_GESTURE_PAPER`。没有创建硬件实例或调用 Init；发布文件只包含导出的数值及来源，未捆绑 SDK 二进制。实物当前固件是 1.1.9，因此动作和单轴写入还需本次实机验证，读取成功不等于动作验证已完成。

锁定提交：`1b552ea77c2d61adef327d179dc9f6b47405c628`，原始 wheel 的 SHA256 记录于 JSON。

- 官方姿态求解器说明：https://github.com/AgibotTech/agillink_omnihand_sdk/blob/main/doc/en/API_KINEMATICS_PYTHON_O10.md
- 锁定版本 SDK：https://github.com/AgibotTech/agillink_omnihand_sdk/tree/1b552ea77c2d61adef327d179dc9f6b47405c628
- 位置写入和故障读取协议（4.3）：https://www.agibot.com/file/ueditor/php/upload/file/20260804/1785833225232972.pdf

离线测试（不会访问 CAN）：

```bash
python3 -B -m unittest discover -s tools/omnihand_can_test/tests -v
```
