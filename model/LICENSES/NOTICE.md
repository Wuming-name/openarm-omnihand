本包新增辅助脚本按 Apache-2.0 提供（全文见 OpenArm_Apache-2.0.txt）。OpenArm 原始模型版权保留 Enactic, Inc.；来源记录 openarm_description commit 73ca62f3e4d04f61d2d1efa112927fe763d4b1f1，见 sources/upstream_provenance.json。

OmniHand 选用本机冻结 SDK 2025 / Python 0.8.0 的右手定义和网格，许可证见 OmniHand_MulanPSL-2.0.txt。SDK README 明确 10 active + 6 passive。本机快照未携带可核验 Git commit，未将它宣称为已在线验证的最新版本。2026.2.1 描述仅作为比较证据，其几何与范围不同，不混用。

已验收组装模型对手部质量作过约 447.1 g 归一化，惯量和质心未实测。commissioning/j7_tool_mount_no_j8.stl 是既有本机派生网格，无独立许可文本；本任务未推定其公开再分发权。衍生凸包、NPZ、USD、OBJ 和 MJCF 继承各原资产的来源与许可限制。

sources/ 原始 URDF 保留历史资源 URI，用于审计和重建，运行入口只使用 ros2_description/ 与 simulation/。不要使用来源证据文件作为独立运行入口。
