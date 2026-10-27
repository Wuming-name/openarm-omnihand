# OmniHand 2025 CAN FD 通信测试（只读）

用于现有 OpenArm + OmniHand 工程，适配 Ubuntu / Python 3.10 / Linux SocketCAN。KH-UCANFD-X2L 已安装驱动且能用于 OpenArm；本工具不依赖品牌专用 SDK，只要求手部所在通道能显示为 SocketCAN 接口。若接口未显示，先核对实际安装的驱动/固件模式，工具不会切换适配器固件。

测试对象是 **OmniHand 2025 / O10 / 十主动轴**，不是 OmniHand Pro O12，也不是达妙电机。工具只会发送三个文档中的读取命令：

| 命令 | 用途 |
|---|---|
| `0xCD` | 查询设备型号和软硬件版本，要求返回 O10 类型 2 |
| `0x02` | 查询当前使能状态，不改变使能 |
| `0x09` | 读取十个主动轴的位置 |

没有使能、失能、校准、运动、写 ID、清零、改波特率或刷固件功能。不读取序列号，不触碰机械臂驱动。读取请求也会向总线发送报文，所以只有 inspect 完全不打开 CAN socket，listen 只接收，probe 才发送读取请求。

## 1. 先检查接口，暂不选通道

在 Ubuntu 项目根目录执行：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
python3 -B tools/omnihand_can_test/read_only_test.py inspect
```

会列出 can 接口、位时序、错误计数、USB 设备及部分 sysfs 通道标识，并保存 JSON。不会打开 CAN socket、发送报文、改变接口或安装软件。

**目前尚未确认 OmniHand 所在物理通道对应 can0 还是 can1。不要凭编号猜测。** 先把 inspect 输出发回，结合适配器插口标识和线缆确认。设备枚举顺序不一定等于物理插口顺序。两个通道看起来配置相同，也不能仅凭软件判断哪个连着手。

若没有 can 接口，可能是尚未加载对应 SocketCAN 驱动、使用了厂商私有 USB 模式或设备未识别；不能据此认定手部损坏。

## 2. 确认手部通道后进行只读查询

**以下 can1 仅为示例，必须替换为实际确认的手部专用通道。** 机械臂继续使用原来已验证的通道，不在本次测试中改动。

```bash
python3 -B tools/omnihand_can_test/read_only_test.py probe \
    --channel can1 --hand-only
```

默认先向文档的分页地址 `0x7FF` 发送一次型号查询，从回复的 CAN ID 识别整手。找到唯一 O10 后，才向该设备读取使能状态和十轴位置。每项最多尝试两次，不扫描全部 ID。

某些固件可能不响应分页查询；已知设备 ID 时显式指定，不能把 SDK 示例默认 ID 当作你设备的真实配置：

```bash
python3 -B tools/omnihand_can_test/read_only_test.py probe \
    --channel can1 --hand-only --device-id 0x09
```

官方手册示例 CAN 默认 ID 为 0x09，部分 SDK 示例使用 0x01；实际以设备设置为准。无需运行智元 SDK 的初始化示例，本工具不会调用初始化中可能附带的动作或使能逻辑。

默认发送带 BRS 的 CAN FD 帧。极少数设备/驱动排查场景可以增加 `--no-brs` 进行不切换数据速率的 FD 查询；不会自动切换、重设接口或尝试经典 CAN。

## 3. 判断是否通过

只有同时收到并解析 **O10 身份** 和 **十轴位置**，才输出 `communication_passed: true`、退出码 0。使能状态为 disabled 也不妨碍只读通信通过。收到自己的回显、仅 socket 发送成功、只有 CAN ACK 或只有使能状态，都不算通信通过。

- 退出码 0：inspect/listen 完成，或 probe 的身份和位置检查通过。
- 退出码 2：参数、接口、权限或底层 socket 错误。
- 退出码 3：未得到完整、唯一、符合文档的 O10 身份和位置应答。

inspect/listen 返回 0 不代表已验证手部通信。报告内同时保留收发原始报文，便于判断不同固件的字段差异。型号/版本表中总长度标注与字段数量不一致，代码采取保守解析；不兼容响应会报告失败，不盲目发送其他协议或运动命令。

十轴位置是 **uint16 原始计数，不是弧度**，文档名义范围为 0～4096。实物软件版本 1.1.9 的只读回复曾出现 4097、4098；工具保留原值，不截断、不归零，并输出 `positions_in_documented_range: false` 和警告。收到完整身份和十轴报文仍可判定通信通过，但超范围原因及这些数值能否用于控制尚未验证。不能直接填入现有 URDF 关节角，后续要根据对应手型、标定和 SDK 做映射。

已验证的一台设备配置：物理 CAN2 接口、Linux can1、设备 ID 0x09，O10 T1，软件 1.1.9、硬件 3.0.0。该设备不响应本次 0x7FF 分页查询，请使用 `--device-id 0x09`。其他设备仍须单独确认配置。

## 4. 可选的短时间接收观察

确认接口后可用以下命令接收 3 秒，不发送应用层查询：

```bash
python3 -B tools/omnihand_can_test/read_only_test.py listen --channel can1 --seconds 3
```

没有帧不等于故障，手部可能仅在收到查询后回复。listen 不会把物理控制器切换成 listen-only 模式，也不会改变已有总线配置。

## 5. 接口配置不正确时

OmniHand 官方参数为仲裁段 1 Mbps、采样点 80%；数据段 5 Mbps、采样点 75%，CAN FD。工具会核验接口 UP、MTU 72 和 1M/5M 位速率，发现异常就停止，不自动 down/up，避免影响通电设备。

**仅在物理通道已确认、该通道只连接手部、且允许暂时停止该手部通道通信时**，可以按以下示例手工配置；can1 仍须替换为实际接口：

```bash
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 sample-point 0.80 \
    dbitrate 5000000 dsample-point 0.75 fd on
sudo ip link set can1 up
```

这组操作只配置电脑 CAN 接口，不改手部存储参数。不要对机械臂正在使用的通道执行。接线和终端电阻按硬件手册确认；OmniHand 手册不允许带电插拔线缆。

通常查询程序无需 sudo；若报权限错误，先保留报告核实原因，不以 sudo 随意运行其他电机测试工具。

## 6. 报告与 Git 管理

报告自动保存于 `outputs/omnihand_can_test/`，由工程现有 gitignore 排除。源码及本说明放在 `tools/omnihand_can_test/`，不修改模型、原有 ROS 展示入口和已存在的本地配置。不要通过 `project.sh display` 包装本工具，展示入口的 AF_CAN 隔离屏障会阻止它访问 CAN。

Windows 笔记本更新源码后，在 CMD 中：

```cmd
cd /d "D:\Projects\openarm_omnihand_gitee_project"
git add tools/omnihand_can_test
git commit -m "Add read-only OmniHand CAN FD communication test"
git push
```

Ubuntu 拉取（普通拉取已配置直连则直接使用）：

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
git pull --ff-only
python3 -B tools/omnihand_can_test/read_only_test.py inspect
```

## 7. 离线测试

两台电脑均可运行，不连接 CAN：

```bash
python3 -B -m unittest discover -s tools/omnihand_can_test/tests -v
```

Windows CMD 将 python3 替换为 python。这些是协议编码、回显排除、应答检查、只读命令限制和模拟通信流程测试；不代表已完成实物通信验收。

## 资料依据

- 智元官方 OmniHand 用户手册，4.1 和 4.3 节（O10、同命令字回复、十轴计数、使能查询和版本查询）：https://www.agibot.com/file/ueditor/php/upload/file/20260804/1785833225232972.pdf
- 鲲弘官方 OpenArm/SocketCAN 接入说明：https://docs.kunhong-elec.com/en/products/can-communication/tutorials/OpenArm.html
- Linux 内核 SocketCAN FD、72 字节结构和本机回显标识：https://www.kernel.org/doc/html/latest/networking/can.html
