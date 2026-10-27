# 资源来源

OpenArm v10 arm/body 网格逐字节复制自本机 `openarm_ros2_v1_ws/src/openarm_description`，保留 Apache-2.0 许可证全文。OmniHand 网格仅使用冻结的 SDK 2025 / 0.8.0 快照，保留 MulanPSL-2.0 许可证全文。

`j7_tool_mount_no_j8.stl` 是既有 commissioning 派生资产，来源和 SHA256 见 `config/model_provenance.yaml`；未找到该派生文件独立的许可证声明，不能据此推定全部内容由本包统一重新授权。本次仅在本机复用，不公开发布。

派生 URDF 保留原始模型几何、惯量及固定手型，仅转换资源 URI、清理无语义字符文本并增加暂定工具坐标系。新增辅助代码按 Apache-2.0 提供；第三方资源仍按各自许可处理。
