# LaserCNC CAM Motion v3.2.1 — B1 / R1 完整审计报告

> 审阅对象：`super1wang/LaserCNCv1.0`  
> 分支：`codex/cam-motion-v3.2.1-b1`  
> 审阅 HEAD：`aa8fd34625bca3581898915b12140f9cc5510cf1`  
> 最新提交：`feat(cam): implement B1 S3 motion reduction`  
> S3 对比基线：`9ca7222efa82f3422cb5e24ae4fc6e3a932fa971`  
> B1 前置基线：B0/R0 已放行的 `54d1eaf593f55711a5965289caee49fe993dead8`  
> 审计日期：2026-09-17  
> **本轮决定：B1 / R1 = PASS_WITH_PATCH；B2_RELEASE = NO。**

## 1. 结论与审计范围

S3 已经完成实质开发：物理轴分析、严格常量轴降维、数值 Z 候选、控制器资格准入、确定性选择、共享 evaluator 派生重建和报告均已接入。不得把它描述为“只有接口”或要求推翻重写。

但 **S3 Soft Check 通过不等于 B1 Stage Gate 通过**。本轮检查 B1 对 B2 的最终交付时，发现两项生产合同仍未闭合：结构性分块与真实工艺事件混用；已发布计划缓存的新鲜度未覆盖新 motion policy。除此之外，仓库实施记录明确表示尚未执行 B1 Stage Gate。三者分别对应 B1-F01、B1-F02、B1-F03。

审阅以固定 SHA 下的主规划、S2 已批准的首版能力边界、S3 实施记录、核心算法、finalizer、生产导出/provider 及测试代码为依据。本文保留已接受的 S1/S2 决策；只有在 B1→B2 完整链路中显现的问题才重新列为交付缺口。审阅是源码与证据审计，不是所有代码的形式化证明。

本轮没有在用户 Windows、ACS/GTN SDK 和实机环境独立运行工程。报告中的 Debug/ASan 结果明确标注为仓库记录；下文反例为源码可推出的路径与建议回归，不冒充已经执行的实机复现。没有修改远端代码、设置、控制器参数或资格状态。

依据：[P1 · B1 主规划：模式、原子发布与最终 Stage Gate](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-cam-motion-compiler_v3_2_1.md#L330-L575)；[P2 · S2 首版能力冻结](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/architecture/pose-space-optimizer_v3_2_1.md)；[P3 · S3 实施及定向验证记录](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-development-execution-steps_v3_2_1.md)；[P4 · B2 Process / GTN 执行边界](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B2-process-gtn-execution.md)。

## 2. 首版能力边界：哪些不能再次作为阻断理由

S2 收口已允许 Full 在缺少生产区间 bound 时执行 Conservative 的严格等价子集，并报告拒绝原因；速度、加速度和反转指标目前是 audit-only。不能在本轮重新要求通用容差平滑或通用 FK 区间证明，才能让 B1 通过。

当前 ACS/GTN 没有可用 qualification authority，生产保持 `Unavailable / revision 0`。这不阻止离线候选构造与 B2 软件开发，但不能由测试 fixture 升级成真实机资格。工艺 Z-hold 没有正式 envelope/whitelist 时，只交付接口、默认关闭和稳定拒绝原因，符合已经批准的首版范围。

因此，以下两句话必须同时成立：

```text
B1 可以交付一个诚实的、控制器无关的 motion compiler。
B1 软件交付不代表所有降维/RTCP 单元已获准上机。
```

依据：[P2 · S2 首版能力冻结](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/architecture/pose-space-optimizer_v3_2_1.md)；[C9 · ReductionPolicy / Admission / ProcessZEnvelope](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.h)；[C10 · 生产 numerical-Z bound 与冻结 FK adapter](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/toolpath_solve_service.cpp#L1-L175)；[P3 · S3 实施及定向验证记录](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-development-execution-steps_v3_2_1.md)。

## 3. B1 规划覆盖矩阵

| 能力 | 本轮判断 | 说明 |
|---|---|---|
| S1 源几何、接缝、几何变更使旧解失效 | 保留已通过结论 | S3 没有改写这套源几何求解基础；最终 Gate 仍需带入直接回归。 |
| 不可变输入及 source/setup/policy identity | 主体接受 | S3 新增 trajectory flags、候选预算与成本版本进入捕获；provider 的缓存失效另见 F02。 |
| 现有 authoritative IK → Optimized Full5D | 接受 | 未新增第二套全局 IK；S3 明确拒绝 Raw reference。 |
| 多圈旋转、入口边与严格 affine 优化 | 按 S2 首版接受 | 不恢复已经关闭的 S2 审计项。 |
| physical active-axis analysis | 接受 | 包含 entry；统计 span、局部变化、travel、反转、freeze residual。 |
| C-only / U+C / 3D / 4D 候选 | 严格等价子集接受 | 枚举物理轴子集，非恒定轴不得遗漏，不按几何圆或单步阈值冻结。 |
| 数值 Z canonicalization | 限定模型下接受 | 64×machine epsilon 的数值预算；边界固定；解析上界可用且 admission 通过才选用。 |
| relaxed process Z-hold | 已声明不启用 | 本轮不要求虚构 envelope 或通用 constrained IK。 |
| Controller admission | 拒绝逻辑接受 | 检查模式、资格指纹/版本、active mask 支持和 dynamics hash；生产仍无可用资格。 |
| 确定性 selection | 主体接受 | 显式字典序与稳定 mask/ordinal；耗时不参加身份。 |
| evaluator 批量派生重建 | 接受 | 块身份检查前移，事务性输出；不以独立缓存插值代替 FK。 |
| 工艺事件语义 | **未闭合：F01** | 每个结构 Block 的末尾都写 laser=false，不能直接作为 B2 工艺事件权威。 |
| 最终缓存及依赖失效 | **未闭合：F02** | 新 policy 的即时导出检查正确，但 public committed provider 仍主要按 geometry revision 取缓存。 |
| 完整 B1 Stage Gate 与 R1 放行 | **未完成：F03** | 只有 S3 定向 7/7 记录，不能替代已约定的批次验收。 |

依据：[C3 · 冻结参数与上下文捕获](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L548-L750)；[C7 · 物理轴分析、资格准入与确定性成本](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L1-L180)；[C8 · 数值 Z、候选构造、派生重建与事务输出](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L175-L321)；[C13 · 连续求值器与批量节点求值](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/continuous_motion_evaluator.cpp)；[T1 · S3 降维与数值 Z 测试](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/dof_reduction_test.cpp)；[T2 · 真实 CamModule 的 policy / stale / publication 回归](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/contour_extraction_test.cpp#L260-L355)；[P1 · B1 主规划：模式、原子发布与最终 Stage Gate](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-cam-motion-compiler_v3_2_1.md#L330-L575)。

## 4. S3 已完成且应保留的实现

### 4.1 严格常量轴降维具有连续证明依据

`analyze()` 覆盖 entry 与全部 physical knots。`movingMask` 使用整个区间的非零 span，而不是逐点小位移阈值。对于 `PhysicalAxisLine`，被固定轴在每个端点恒等，足以说明分段 affine 插值中的该轴持续恒定。因此这个严格子集不必依赖未知的通用 TCP bound 才能成立。

测试中，1001 个每步 `1e-9` 的 X 增量被保留为真实累计运动；C-only、U+C、3D、4D 和不合格候选回退都有覆盖。这里的资格对象明确只存在于测试 fixture。

依据：[C7 · 物理轴分析、资格准入与确定性成本](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L1-L180)；[C8 · 数值 Z、候选构造、派生重建与事务输出](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L175-L321)；[T1 · S3 降维与数值 Z 测试](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/dof_reduction_test.cpp)。

### 4.2 完整物理位姿与参与轴承诺已分开

节点 `axisMask` 继续保存完整 physical layout；Block `activeAxisMask` 才表达本段需参与运动的物理轴。非活动轴不是“数据缺失”，而是必须持续保持已存储的值。B2 不能把“字段没有下发”自行解释成保持当前反馈位置。

finalizer 已对 `Reduced` 状态检查资格和 inactive-axis hold；它是正确增量，但完整的状态/掩码一致性还可按 O02 加固。

依据：[C9 · ReductionPolicy / Admission / ProcessZEnvelope](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.h)；[C12 · FinalMotionPlan finalizer / 身份检查](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/project/cam/final_motion_plan.cpp#L205-L344)；[T1 · S3 降维与数值 Z 测试](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/dof_reduction_test.cpp)。

### 4.3 数值 Z 与工艺 Z 没有混为一谈

数值预算为：

```text
64 × machine epsilon × max(1, abs(anchor))
```

这不是人工新增的工艺离焦容差。末点不等于 anchor 时拒绝数值 Z，以免改变下一 Block 的入边。生产 bound 仅支持已检查拓扑的 planar/table 独立 linear Z 平移，不推断 head、缩放安装或 Z 带动工件/旋转轴的情况。

尚无 process envelope 时，即使 `enableLaserZHold=true`，也只记录缺少资格的原因，不执行工艺 Z-hold。此边界符合首版约定。

依据：[C8 · 数值 Z、候选构造、派生重建与事务输出](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L175-L321)；[C10 · 生产 numerical-Z bound 与冻结 FK adapter](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/toolpath_solve_service.cpp#L1-L175)；[C9 · ReductionPolicy / Admission / ProcessZEnvelope](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.h)。

### 4.4 资格与确定性处理是有效代码，不是 UI 装饰

`CamConfig.[trajectory]` 的 Off/Conservative/Full 和两个 feature flags 已进入 frozen identity；错误类型和未知模式会导致编译拒绝。准入检查不把 SDK 存在、构建通过或 fixture 当成资格；生产未资格时回到原 Optimized reference。

字典序成本按 stops、duration、limit proximity、rotary reversals、normalized travel、knot count、active-axis count、mask、ordinal 比较。算法耗时只进诊断报告，不进入 planHash。成本中的 stop 统计仍受 F01 的事件语义问题影响。

依据：[C6 · trajectory 参数解析](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/settings/cam_config.cpp#L215-L235)；[C3 · 冻结参数与上下文捕获](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L548-L750)；[C7 · 物理轴分析、资格准入与确定性成本](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L1-L180)；[T2 · 真实 CamModule 的 policy / stale / publication 回归](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/contour_extraction_test.cpp#L260-L355)。

## 5. 阻断项 B1-F01 — P1：结构性分块被写成工艺开关事件

### 5.1 代码事实

`attachMotionPlan()` 会因 source edge 改变、departure alias、semantic barrier、phase/contour 改变而新建 Block。每个 Cutting Block 都加入 laser=true 的开始 fence，而所有 Block 的结束 fence 均硬编码为 laser=false：

```cpp
block.fences.append({0, true, false,
    node.phase == CamMotionPhase::Cutting});
// ...
block.fences.append({lastKnot, false, true, false});
```

`MotionProcessFence` 字段名明确是 `laserEnabledAfterFence`，没有“仅结构标记、不修改工艺状态”的独立表达。S3 `selectionCost()` 又把该字段为 false 的 fence 累加为 stops。

证据：[C1 · Raw Block 分块与 fence 构造](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L870-L980)；[C11 · MotionProcessFence 与 SourceSpan 数据合同](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/project/cam/collision_validation_contracts.h#L236-L270)；[C7 · 物理轴分析、资格准入与确定性成本](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L1-L180)。

### 5.2 可构造的语义反例

输入是同一条连续 Cutting 轮廓，由两条切向连续 OCC edge 组成，没有人工 stop、laser-off 或工艺切换。

```text
原工艺：LaserOn → edge0 → edge1 → LaserOff

当前结构分块：
Block0 start=true → edge0 → Block0 end=false
Block1 start=true → edge1 → Block1 end=false
```

按现有字段作为工艺状态解释，源边接缝额外出现 Off→On。当前 Process 尚未完成 B2 cutover，所以本文不声称现场已经按此反复开关激光；问题在于 B2 被要求原样执行 FinalMotionPlan process fences，却没有合法依据判断哪些是可丢弃的结构标记。

### 5.3 风险与目标

风险是额外 IO 切换、被后续实现误加停止边界，以及 selection cost 把结构划分当成工艺停止。即使 B2 在同一位置合并相邻事件，也必须先知道该 Off 是否真实工艺要求，不能靠猜测。

收口必须分开：

```text
source span / block boundary / optimizer barrier
≠ laser state transition
≠ required mechanical stop
```

保持真实工艺事件的原位置、次序与状态。纯 source seam 可继续分块，但不得凭分块自动创造 LaserOff。机械 stop 的成本应来自显式 stop 语义，不能用 `!laserEnabledAfterFence` 代替。

### 5.4 最小验收

一条两边连续轮廓：Raw→Full5D→Reduction 后真实 laser/stop 事件序列不增加。另一个有显式 LaserOff/stop 的样例必须完整保留；切换 mode、切分 source span、数值 Z 或降维不能改变事件含义。测试使用无设备事件投影即可，不要求 GTN 实机。

## 6. 阻断项 B1-F02 — P1：直接导出已验证新 policy，已发布缓存没有同等失效合同

### 6.1 代码事实

直接 `attachMotionPlan()` 会在前后比较 `sameMotionAuthority()`；新策略能进入 `contextHash`，相关直接调用测试也会拒绝旧输入。这些检查本身正确。

但外部 `ICamToolpathProvider::exportCommittedExecutionSnapshot()` 经过 adapter 后，只在计划与目录的 `revision` 相等时返回 `m_plannedSnapshot`；本 getter 未比较 motion policy/context，未比较统一 publication epoch。adapter 的刷新连接列举了现有 CAM 事件，没有新 trajectory-policy 专用变更通知。`CamModule::config()` 则公开可变 `CamConfig&`，配置重读可改变新策略，而不必改变几何刀路 revision。

证据：[C2 · Full5D → Reduction → 最终发布检查](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L985-L1065)；[C3 · 冻结参数与上下文捕获](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L548-L750)；[C4 · Committed provider 缓存与刷新](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/integration/cam_service_adapters.cpp#L37-L165)；[C5 · CamModule 的公开可变 config 接口](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/cam_module.h#L135-L145)；[C6 · trajectory 参数解析](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/settings/cam_config.cpp#L215-L235)。

### 6.2 可构造的链路反例

```text
1. 按 policy A 生成并缓存计划 P，geometry revision=r。
2. 通过公开配置重读把 trajectory 改为 policy B；不重建几何。
3. 直接 captureMotionCompilationInput 可看出 A/B authority 不同。
4. 通过 public committed provider 读取：只要 r 未变，仍返回缓存 P。
```

本轮没有在 GUI 中执行这条用例；这是公开可变配置入口与 provider 选择条件共同支持的静态反例。现有测试在修改配置后直接调用 `attachMotionPlan()`，没有验证同样变化后通过注册 provider 读缓存的行为。因此不能拿直接导出的 stale 测试代替公共服务合同测试。

证据：[T2 · 真实 CamModule 的 policy / stale / publication 回归](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/contour_extraction_test.cpp#L260-L355)；[C4 · Committed provider 缓存与刷新](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/integration/cam_service_adapters.cpp#L37-L165)。

### 6.3 为什么应在 B2 前关闭

B2 要消费“已发布的精确计划”，而不是自己重跑 CAM 检查并选择新候选。读取缓存的过程必须能区分：当前已发布计划、旧计划仅供显示、当前计划待编译/已失效。

现有互斥锁能避免读取半个 snapshot，但不能说明 snapshot 对当前 policy 仍然有效。二者必须分别保证。也不能以已有 `planHash` 自洽代替外部依赖未变。

### 6.4 最小修复方向

复用现有 adapter/cache，不建立第二个可写 motion plan。统一跟踪 source/order/context/policy/capability 的 publication key 或 owner 管理的变更 epoch；所有相关配置应用/重读入口必须先使旧执行资格失效。getter 只返回与当前授权 key 匹配的 committed snapshot，否则返回明确的 NotReady/Stale，不把 raw catalog 当作新的已提交计划。

跨线程 getter 不得持 cache lock 阻塞调用 GUI/owner，也不得直接读取可变 UI。优先由 owner 原子更新依赖 epoch 和不可变 published record。相同 key 的重复读取不得重新运行 Full5D/Reduction。

### 6.5 最小验收

必须通过实际注册的 `ICamToolpathProvider`：冻结旧计划，改变 trajectory mode/feature flag 或其他相关 authority，确认旧计划不能再以 current committed 状态返回；重新编译后才恢复。保留取消/迟到 worker 拒绝和 exact-hash collision attachment 回归。

## 7. 阻断项 B1-F03 — 阶段证据：尚未执行原规划规定的 B1 Stage Gate

仓库 S3 实施记录明确表示：本次是 S3 Soft Check，尚未执行 B1 Stage Gate。记录了 Debug 和 ASan 定向各 7/7，不能据此把 V-007～V-016 的完整 B1 验收自动标为通过。

本项不是算法错误，也不是要求再加一层重复验收。只需要完成原本就存在的一次 B1 Stage Gate，并把本次 F01/F02 回归合并进去。相同测试可同时支持多个验证条目，不要求逐条再跑。

V-010/V-011/V-014 应如实反映已批准的首版范围：严格等价子集通过、无权威 bound/工艺 envelope 的功能保持关闭；禁止为了满足旧矩阵文字伪造资格，也禁止把未实现的扩展能力写成全能力 PASS。

要求一份汇总证据：固定 commit、实际命令、test→V-row 映射、before/after 指标、生产 unqualified 与 test-only qualification 分列、失败/重跑说明、Deferred 清单，以及 R1 决定。不得使用 S1/S2 的旧 commit 测试覆盖改动后的最终接口而不说明影响。

依据：[P3 · S3 实施及定向验证记录](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-development-execution-steps_v3_2_1.md)；[P1 · B1 主规划：模式、原子发布与最终 Stage Gate](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-cam-motion-compiler_v3_2_1.md#L330-L575)；[P5 · 批次验证矩阵 V-007～V-016](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/validation/validation-matrix.md)。

## 8. 有价值的优化与非阻断加固

### O01 — 将最终优化从 owner 导出重计算收敛为一次编译、一次发布

现在 `attachMotionPlan()` 在 owner 导出路径同步运行 Full5D 和 Reduction；生产调用没有传入算法可选的取消回调。已有 adapter 确实缓存结果，不能说“完全没有缓存”，但 `refreshPlannedCacheForOrder()` 在尝试合并 proof 之前仍重新调用生成完整计划的导出路径。

建议复用现有 TaskManager，把长耗时优化放到持有不可变输入的编译任务；owner 只做最终依赖检查和短临界区替换。同 key 的 proof 更新只更新附件。不要另造 SDK worker 或新的设备队列。

这是高收益的吞吐、响应性和取消能力改进，不以本轮未测量的毫秒数或加速倍数做结论。若本次采用最小缓存失效修复，余下性能重构可明确排期；但最终 V-016 仍须证明实际采用的取消路径不会发布过期结果。

依据：[C2 · Full5D → Reduction → 最终发布检查](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L985-L1065)；[C4 · Committed provider 缓存与刷新](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/integration/cam_service_adapters.cpp#L37-L165)。

### O02 — finalizer 应验证语义，而不只根据 optimizationState 选择检查

当前 inactive-axis hold 检查只在 `optimizationState == Reduced` 时执行。构造一个完整轴位姿中 X 正在变化、Block activeMask 仅 C 的对象，再把标签改成 Optimized，现有该分支就不做 hold 检查。当前 reducer 正常输出不会这样做；这是 finalizer 边界反例，而不是已观察到的正常降维错误。

建议统一校验 active mask 与 physical layout、motion class、逐轴 hold 的一致性；区分真正三轴机布局和五轴机降维，不能粗暴要求所有 <5 轴的 Block 都有降维资格。source span 范围和 departure alias 也应验证必要的结构约束，但允许已经定义的 arrival/departure 元数据重叠，不能以“所有 span 绝不重叠”破坏 S2 合同。

建议同批低成本加固，不单独增加 Stage Gate。

依据：[C12 · FinalMotionPlan finalizer / 身份检查](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/project/cam/final_motion_plan.cpp#L205-L344)；[C9 · ReductionPolicy / Admission / ProcessZEnvelope](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.h)。

### O03 — 继续降低 evaluator / candidate 的重复计算

本次 `evaluatePhysicalKnots()` 已将逐点全块哈希改为一次块校验、批量求精确 knot，值得保留。仓库报告的 1001 点 ASan 用例由超时变为通过，只能按记录描述，不能外推整机加速倍率。

仍可优化：每个 Block 的 `bindMotionEvaluationContext()` 会重复验证整份 candidate plan；生产 FK callback 每个点重建 `MachineKinematics`；候选重复进行全块 analyze/cost。优先建立不可变计划会话、worker 私有 FK 状态、复用不随 mask 改变的成本项。不得删掉身份保护换性能。

依据：[C13 · 连续求值器与批量节点求值](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/continuous_motion_evaluator.cpp)；[C8 · 数值 Z、候选构造、派生重建与事务输出](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L175-L321)；[C10 · 生产 numerical-Z bound 与冻结 FK adapter](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/toolpath_solve_service.cpp#L1-L175)；[P3 · S3 实施及定向验证记录](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-development-execution-steps_v3_2_1.md)。

### O04 — 保留既有 Deferred，不反复扩大 B1

直接按 `ceil(delta/maxStep)` 分段、最大 affine span merge、真实 controller lowering/native primitives，以及更多 Z 工艺白名单均有潜在收益，但不是本轮新增门禁。生产无资格仍不执行 Reduced，不应被当成“算法没有实现”；测试 fixture 不能当现场 qualification。

同时修正文档仍写“未提交/未推送”的旧事实，并保留配置 requested/effective/source 信息。耗时、命令数估算与真实控制器周期必须区分，不把 knot 数下降直接宣传成抖动已经改善。

## 9. 验证证据与限制

| 证据 | 来源与判断 |
|---|---|
| S3 Debug 定向 7/7，记录 17.02 秒 | 仓库实施记录；本轮未独立执行。 |
| S3 ASan 定向 7/7，记录 42.04 秒 | 仓库实施记录；本轮未独立执行。 |
| 1001 点批量求值优化，记录 Debug 0.48 秒、ASan 1.31 秒 | 仅该测试记录，不外推全工程或实机性能。 |
| 参数错误、未资格、过期资格、取消、数值 Z 边界、tiny increments | 已检查相应测试源代码。 |
| F01 连续多 source-edge 的事件不增生 | 未看到完整生产链回归，应补。 |
| F02 policy 改变后通过已注册 provider 读取 | 现有直接 attach 测试不足，应补。 |
| B1 Stage Gate | 仓库明确尚未执行。 |
| 该 SHA GitHub commit statuses / Actions runs | 本轮查询未返回独立结果。没有 CI 结果本身不阻断本地开发 Gate。 |
| GTN/ACS 实机、激光、HIL、生产碰撞资格 | 不在本轮范围，也没有本轮验证结论。 |

依据：[P3 · S3 实施及定向验证记录](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-development-execution-steps_v3_2_1.md)；[T1 · S3 降维与数值 Z 测试](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/dof_reduction_test.cpp)；[T2 · 真实 CamModule 的 policy / stale / publication 回归](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/contour_extraction_test.cpp#L260-L355)；[P5 · 批次验证矩阵 V-007～V-016](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/validation/validation-matrix.md)。

## 10. 最终放行矩阵

| 事项 | 决定 |
|---|---|
| S1/S2 已接受的首版能力 | 保留，不推翻、不重新开整批验收。 |
| S3 核心降维、数值 Z 与资格拒绝实现 | 接受主体，保留。 |
| B1 整体完成 | **PASS_WITH_PATCH**。 |
| B2 正式开发放行 | **NO；F01/F02 闭合并完成原 B1 Gate 与 R1 后转 YES。** |
| 是否需要重做 B1 | 不需要。一个连续收口批次即可。 |
| 是否因没有 ACS/GTN 资格而阻塞 B1 数学交付 | 不应如此。缺资格必须真实保留。 |
| 真实设备执行资格 | 本轮不放行；继续受后续 qualified cell / B2～B4 门禁约束。 |

执行入口：[B1 R1 收口执行规划](B1_R1_CLOSEOUT_EXECUTION_PLAN.md)。

## 11. 固定 SHA 证据索引

以下链接均锁定本次审阅 HEAD。正文中的缺陷为源码事实及明确标记的推论；修复方案是本次建议，不伪装成原代码已经具备的能力。

- [P1 · B1 主规划：模式、原子发布与最终 Stage Gate](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-cam-motion-compiler_v3_2_1.md#L330-L575)
- [P2 · S2 首版能力冻结](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/architecture/pose-space-optimizer_v3_2_1.md)
- [P3 · S3 实施及定向验证记录](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-development-execution-steps_v3_2_1.md)
- [P4 · B2 Process / GTN 执行边界](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B2-process-gtn-execution.md)
- [P5 · 批次验证矩阵 V-007～V-016](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/validation/validation-matrix.md)
- [C1 · Raw Block 分块与 fence 构造](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L870-L980)
- [C2 · Full5D → Reduction → 最终发布检查](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L985-L1065)
- [C3 · 冻结参数与上下文捕获](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L548-L750)
- [C4 · Committed provider 缓存与刷新](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/integration/cam_service_adapters.cpp#L37-L165)
- [C5 · CamModule 的公开可变 config 接口](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/cam_module.h#L135-L145)
- [C6 · trajectory 参数解析](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/settings/cam_config.cpp#L215-L235)
- [C7 · 物理轴分析、资格准入与确定性成本](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L1-L180)
- [C8 · 数值 Z、候选构造、派生重建与事务输出](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L175-L321)
- [C9 · ReductionPolicy / Admission / ProcessZEnvelope](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.h)
- [C10 · 生产 numerical-Z bound 与冻结 FK adapter](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/toolpath_solve_service.cpp#L1-L175)
- [C11 · MotionProcessFence 与 SourceSpan 数据合同](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/project/cam/collision_validation_contracts.h#L236-L270)
- [C12 · FinalMotionPlan finalizer / 身份检查](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/project/cam/final_motion_plan.cpp#L205-L344)
- [C13 · 连续求值器与批量节点求值](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/continuous_motion_evaluator.cpp)
- [T1 · S3 降维与数值 Z 测试](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/dof_reduction_test.cpp)
- [T2 · 真实 CamModule 的 policy / stale / publication 回归](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/contour_extraction_test.cpp#L260-L355)
- [T3 · Full5D 数值与入口边回归](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/tests/full5d_optimizer_test.cpp)
- [T4 · 状态矩阵](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/STATUS-MATRIX.md)
