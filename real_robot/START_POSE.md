# 初始姿态碰撞诊断

如果commission_hold在模型碰撞检查处退出且hardware_commands_attempted=false，说明没有尝试使能或发送MIT力矩；不应反复使能或跳过碰撞检查。

用户2026-10-28日志显示近零关节位置下，训练模型检测到thumb_mcp/pip/dip与底座几何穿透9～18mm，以及link5/link7约2.55mm穿透。这些是模型预测，不等于实物必定接触。需区分真实接触、关节零点/符号映射差异、固定手部姿态差异，以及凸包碰撞近似误报。环境四周空旷不能排除自身底座/腕部接触。

退出运动控制程序后只读测量并生成四视图，图的上排是最新测量角度映射到训练模型的预测，下排是训练A点。红色小球标记超过任务配置穿透容差的接触点，模型原有红/绿目标标记也可能显示。手部保持训练模型冻结姿态，图片不是实物照片。

```bash
cd /home/lsk/openarm_v1_ws/src/openarm-omnihand-project
source .venv-ppo/bin/activate
python -m real_robot.probe
# 上一步成功后60秒内运行，使用刚生成的七轴报告；不打开CAN
python -m real_robot.inspect_start \
  --run outputs/ppo/run_20261028_164711_538719
```

生成`outputs/real_robot/start_pose_comparison.png`及同名JSON。图形渲染需要可用OpenGL上下文；无显示环境请在Ubuntu图形桌面终端运行。离线查看指定历史probe可用`--probe PATH`，此时不会检查文件60秒新鲜度，需要操作者确认测量时刻。

不要修改训练task/config/model来绕过检查；这些文件已被策略契约校验。也不要在未验证初始路径时执行零位直达A。应先将图片与实物结构和姿态对照，之后再决定标定修正、碰撞近似修正或经验证的无碰撞起始姿态。

commission_hold现在会在模型预检查之前把七轴实测位置、速度及模型SHA写入最终报告，即使碰撞拒绝也保留输入，方便复现。
