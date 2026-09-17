# B1.S2 收口记录

- 基线 HEAD：`e8428b6666be929df3b090b57e0bd1686ef28172`；S2 收口已提交为 `9ca7222`。
- 目标仓库：`https://github.com/super1wang/LaserCNCv1.0.git`，分支 `codex/cam-motion-v3.2.1-b1`；未推送。
- 范围：仅 C2.1 / C2.2 / C2.3，不进行 B1 Stage Gate，不实施 S3。

## 修复

- C2.1：接缝保留到达/离开两侧原生源身份。反向交换两侧、旋转起点随点携带，闭合执行点数不增加；Raw 在 departure 后拆 block，并表达 `-1 → 0` 源区间。别名贯通导出、source fingerprint、plan identity 和 TOML 持久化；携带别名时使用 geometry proof version 3，旧读取器不得忽略后仍资格化。
- C2.2：incoming 同源边按 departure/native 参数细分；跨源、跨 phase、Rapid 使用 owner `-1` 的 motion-edge `[0,1]` 参数，不冒充 OCC。插入点属于当前 block，保留 predecessor、entry fence `-1` 和源 span，重新生成 identity；失败/取消仍事务性拒绝。
- C2.3：首版 Off / Conservative / Full 能力已同步至主规划、执行步骤与 pose 设计。生产 bound 不可用时 Full 执行 Conservative 等价子集并记录确定性拒绝原因；非零 smoothing 不启用。速度/加速度 proxy 和反转计数仅审计，测试 bound 仅基础设施，不代表生产资格。

## 验证

Debug 完整构建通过；架构检查通过。定向回归首轮 5/6，修正新增跨 phase 测试在 finalization 后引用旧 QVector 元素的问题后，失败项复跑 1/1 通过，累计六项全通过：

```text
lcnc_full5d_optimizer_test
lcnc_cam_algorithm_pipeline_test
lcnc_cam_motion_foundation_test
lcnc_cam_motion_plan_contract_test
lcnc_cam_lead_in_test
lcnc_project_package_test
```

新增覆盖闭合圆/多边形、反向、自动/手动起点、原生周期方向、post-IK → Raw → Optimized、生产 departure span、持久化、incoming `C=0→30° / step≤5°`、同源/跨源/跨 phase/rapid phase、Full 缺失 bound 的确定性行为及 dynamics audit-only。

ASan 独立配置完整构建通过，同组六项回归 **6/6 通过**（40.91 秒），无 sanitizer 报错。
未重复全量架构审计/Stage Gate；本次仅执行 S2 Soft Check 与直接回归。

## 延后与放行边界

- O2.1 evaluator 热路径复用、D1 直接分段数、D2 最大 affine 区间合并：Deferred，不阻塞 S3。
- controller 保持 Unavailable/revision 0，生产 policy 仍 Off；RTCP/controller admission 继续关闭。
- 未进行 HIL、实机或 production collision 验收；本记录不能代替这些资格。
- C2.1 / C2.2 / C2.3 均关闭，无未解决 S2 ESCALATE。
- **B1.S2 = PASS；S3_RELEASE = YES**。S3 可消费 Optimized Full5D 开发，但未资格 controller/RTCP admission 仍关闭；本轮未实施 S3。
