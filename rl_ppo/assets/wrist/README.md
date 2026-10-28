# 腕部碰撞部件

来源为现有 OpenArm 模型的 link5/link7 碰撞 OBJ。脚本将原三角形按连通部件拆开；独立平面补片并入最近的立体部件，保留全部输入几何。MuJoCo 对每件分别使用凸包。没有增加排除对，没有缩小网格。

`manifest.json` 记录输入与输出 SHA256、工具版本和每处部件数。重建入口是 `python -m rl_ppo.decompose`，详情见上级代码说明。原模型与授权位于工程 `model/LICENSES/`，OpenArm 原始几何授权为 Apache-2.0。这里只包含派生几何，不包含 CoACD 或 trimesh 源码。

这仍是近似碰撞模型，需要在视觉与实物尺寸上复核；仿真接触检查通过不等于实机安全认证。
