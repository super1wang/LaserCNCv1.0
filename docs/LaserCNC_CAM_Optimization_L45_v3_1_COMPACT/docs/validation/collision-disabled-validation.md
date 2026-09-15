# 无机台几何 / Collision Disabled 验收方案

## Objective

证明系统只凭运动学配置即可完成轨迹生成、优化、发布、仿真和控制器下发准备，不会被机台 STEP/安全包/碰撞证书强制阻断。

## Fixture

- 有效 AC/BC/Head machine kinematic config；
- machine geometry path 为空或未加载；
- `.lmsp/.lmsi` 不存在；
- Job Overlay 不创建；
- `collision.verificationMode=Disabled`；
- 一个 Planar、一个 RotaryTube、一个 FullTable5D 测试刀路。

## Assertions

1. CAM pipeline 不报告 machine package missing；
2. optimizer 正常输出 Final Motion Plan；
3. collision backend construction/query count = 0；
4. Process `camExecutionBlockReason` 不因 proof 缺失阻断；
5. controller disconnected/fault/axis disabled 仍阻断；
6. RTCP target invalid 仍阻断；
7. UI/log 明确 `collision=disabled/not-certified`；
8. 切换为 Required 后同一环境必须 block，不能沿用 Disabled eligibility。

## Exit

以上全部通过，才能宣称“轨迹规划不被强制引入碰撞检测”。
