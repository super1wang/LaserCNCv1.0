# LaserCNC CAM Motion v3.2.1 — B1 / R1 收口执行规划

> 目标分支：`codex/cam-motion-v3.2.1-b1`  
> 审计基线：`aa8fd34625bca3581898915b12140f9cc5510cf1`  
> 当前：`B1/R1 = PASS_WITH_PATCH`；`B2_RELEASE = NO`  
> 目标：关闭 B1-F01 / B1-F02，完成既定 B1 Stage Gate，R1 复核通过后放行 B2。  
> **执行方式：一个连续收口批次；中间不设人工验收；最终一次 B1 Gate + 一次 R1 复核。**

## 1. 范围与不可扩大项

本批次不是重做 S1/S2/S3。保留当前 authoritative IK、几何 proof、departure alias、entry refinement、strict Full5D、物理轴降维和数值 Z。

必须开发：

```text
C1 结构边界 / 工艺状态 / required stop 分离
C2 最终已发布计划的 authority 失效与 provider 一致性
C3 合并执行原有 B1 Gate，补齐证据与状态
```

同批建议但不另设 Gate：finalizer 语义加固、重复计算削减、文档事实清理。

不要求：通用非零姿态平滑、生产通用区间 bound、process Z whitelist 发明、GTN 实机、HIL、完整碰撞认证、B2 lowering 提前实现。无 qualification 继续 Unavailable/revision 0，不伪造资格；B2 开发允许使用明确隔离的 test-only fixtures。

依据：[审计报告](B1_R1_AUDIT_REPORT.md)；[P1 · B1 主规划：模式、原子发布与最终 Stage Gate](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B1-cam-motion-compiler_v3_2_1.md#L330-L575)；[P2 · S2 首版能力冻结](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/architecture/pose-space-optimizer_v3_2_1.md)；[P4 · B2 Process / GTN 执行边界](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B2-process-gtn-execution.md)。

## 2. 全批次冻结不变量

| ID | 必须保持 |
|---|---|
| INV-01 | FinalMotionPlan 仍是唯一正式 motion truth；旧 point/catalog 不能反向回写它。 |
| INV-02 | 同一真实工艺事件的位置、顺序、状态不能被 source 分块、merge、降维或数值 Z 改变。 |
| INV-03 | `entryBoundary` 是唯一 predecessor，不投影为重复 execution node。 |
| INV-04 | 多圈 physical rotary、arrival/departure source identity 和 source traversal 保持。 |
| INV-05 | 任何改变仍受同一 frozen kinematics/evaluator、软限位和数值/工艺预算约束。 |
| INV-06 | B1 optimizer 不依赖 collision 后端；未资格 controller 不能取得真实准入。 |
| INV-07 | 成功计算不等于可发布；发布必须匹配当前 owner authority / order / workspace / policy。 |
| INV-08 | public committed provider 不能把过期计划或 raw catalog 冒充当前 committed plan。 |
| INV-09 | 取消、stale 或失败不发布部分结果；同 key 重复读取不重新选择运动。 |
| INV-10 | 不新增设备队列，不在 Process 重新 IK、简化或切换候选。 |

## 3. 开发准备：只确认本次增量

读取本报告、当前主规划末段、B2 消费边界，以及下列实际文件。不要重建本地环境，不做整仓库重新探索。

```text
src/modules/cam/toolpath/cam_module_toolpath.cpp
src/modules/cam/integration/cam_service_adapters.cpp
src/modules/cam/settings/cam_config.h/.cpp
src/core/project/cam/collision_validation_contracts.h
src/core/project/cam/final_motion_plan.cpp
src/core/algorithms/cam/dof_reduction.cpp
src/core/algorithms/cam/continuous_motion_evaluator.cpp
相关 pipeline / TaskManager 现有入口
```

若分支已超过审计 SHA，先列出影响上述边界的真实 delta。已有修复直接复用，只补缺项；不因 SHA 增加而重新实施整个规划。

## 4. C1 — 分离结构 fence 与工艺事件

### 4.1 目标与输入

目标：同一连续激光加工轮廓，不因换 sourceEdge、departure seam 或结构 Block 而凭空增加 LaserOff/LaserOn/Stop。

输入：已有 phase/contour/source/fence 语义和现有工艺流程。工艺控制时序必须来自已有权威；本文不重新规定 lead-in 何处开光、穿孔延时多长或何时开气。

源码事实：[C1 · Raw Block 分块与 fence 构造](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L870-L980)；[C11 · MotionProcessFence 与 SourceSpan 数据合同](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/project/cam/collision_validation_contracts.h#L236-L270)；[C7 · 物理轴分析、资格准入与确定性成本](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.cpp#L1-L180)。

### 4.2 连续开发步骤

**第一步：固定三种边界语义。** 将结构分块/优化屏障、激光状态变更、机械 required stop 分开。优先对现有 MotionProcessFence 作最小语义扩展，不另建一套独立执行事件计划。

允许的实现是显式 event kind / optional state change / stop flag，或继续使用状态型字段但对结构边界继承真实状态。关键是 consumer 无需猜测 `false` 是“无事件”“关光”还是“需要停机”。跨 source span 仍可保守分块，不要求本次跨源最大 merge。

**第二步：修正 Raw builder。** 删除“每个 Block 末尾都表示 laser=false”的泛化规则。只在真实工艺事件处改变状态；纯 source split 的前后状态连续。保留 blockStart/blockEnd 和 entry `-1` 的结构位置，但结构标记不等于设备 IO。

**第三步：调整 optimizer 屏障与成本。** Full5D/Reduction 继续保护真实事件与不可跨越屏障；不得仅因新增 event 分类就放宽几何 corner 或 source ownership。`selectionCost()` 只统计真正 required stops；不能再用 laser=false 的数量代替停机数。成本版本变化必须进入 frozen policy/hash。

**第四步：更新 finalizer 与身份。** 新增的工艺语义进入 node/block/plan identity。校验事件索引、事件在入边之前还是 knot 处、前后状态和真实事件顺序。数据格式变化按现有版本机制处理，旧数据无法明确解释时不得自动当成新格式有效计划。

**第五步：增加无设备事件投影测试。** 将 Raw、Optimized 和 Reduced 的真实工艺事件投影成规范序列，只比较真实状态变更/required stop，不把结构 marker 算成硬件命令。

### 4.3 禁止修法

不得让 B2 “看到相邻 Off/On 就自行删除”；其中可能有合法工艺语义。不得把所有 fence 都删掉；不得为避免事件误切换而把整个轮廓塞成一个无法表达 source mapping 的 Block。不得临时创造激光工艺配方。

### 4.4 完成条件

同一连续双源边轮廓的事件投影在 Raw→Full5D→Reduction 之间不增生；有真实 Off/Stop 的反例仍完整保留；修改真实事件改变 identity，纯结构边界不能伪装成工艺停止成本。

## 5. C2 — 统一 current authority、已发布 snapshot 与缓存失效

### 5.1 目标与输入

目标：直接导出和通过注册 `ICamToolpathProvider` 读取，对“哪个计划当前有效”给出相同答案；不把几何 revision 当作所有语义依赖的代理。

复用现有 `MotionCompilationInput`、owner 捕获、TaskManager 和 adapter mutex cache。现有缓存有原子 copy 能力，应保留；需要补的是依赖身份和失效路径。

源码事实：[C2 · Full5D → Reduction → 最终发布检查](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L985-L1065)；[C3 · 冻结参数与上下文捕获](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/toolpath/cam_module_toolpath.cpp#L548-L750)；[C4 · Committed provider 缓存与刷新](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/integration/cam_service_adapters.cpp#L37-L165)；[C5 · CamModule 的公开可变 config 接口](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/cam_module.h#L135-L145)；[C6 · trajectory 参数解析](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/modules/cam/settings/cam_config.cpp#L215-L235)。

### 5.2 连续开发步骤

**第一步：定义 publication key。** 基于现有 authority/source/order/workspace/context identity 构造单一 current key 或 owner epoch。区分已捕获输入、实际已解算顺序和最后成功发布计划，不另建与 ToolpathGenerationStamp 平行的 source revision。

**第二步：收拢所有相关配置应用入口。** trajectory 模式、feature flags、policy budget/cost revision，以及会影响结果的机台/setup/工具/控制器资格变化，都必须使旧计划的执行资格失效。特别覆盖公开 `config()` 后的重读/应用路径；可以增加配置变更通知或统一 owner 应用服务，但不能只为测试的直接 attach 路径补检查。

若继续暴露可变配置引用，必须保证其受支持 mutation/load 路径能参与失效；否则限制该入口，并迁移现有调用者。禁止依靠“调用者记得手动刷新”。

**第三步：发布一个完整不可变 record。** 至少包含 FinalMotionPlan、对应 input/context key、实际顺序、report 与 readiness。可扩展原 ToolpathExportSnapshot，不必建立第二个可写 plan。失败/取消时保留旧计划用于显示是允许的，但其执行状态必须明确 Stale/NotReady。

**第四步：修正 public provider。** `exportCommittedExecutionSnapshot()` 只返回 key/epoch 匹配的 committed record；旧目录/points 仍可由 catalog/display API 使用，但不能作为“当前 committed plan”的无声 fallback。background reader 不直接读取 mutable CamConfig 或 UI；不得持缓存锁阻塞调用 owner 造成死锁。

**第五步：把本次采纳规则与异步任务关联。** late worker 必须持原输入 key，owner 对比当前 key 后再一次替换。相同 key 的重复读不触发 optimizer。取消必须通过实际任务/令牌覆盖最终 selection，不能只有纯算法 fixture 能取消。

建议顺带把 Full5D/Reduction 从同步导出重计算迁入现有编译任务，以消除主线程阻塞和 proof-refresh 重算。如果保持同步实现，仍必须交付上述 freshness、只读缓存和真实取消/失效语义，并在 Gate 记录响应性限制；不因此另建新调度系统。

**第六步：限定 proof 更新。** 相同 exact planHash 的 collision 附件可独立更新；不得通过 proof-only 通知重新选择 motion candidate。碰撞 Required/Optional/Disabled 的已批准语义保持不变。

### 5.3 必须采用的生产入口回归

测试先从注册 service registry 获得 `ICamToolpathProvider`，而不是只调用 CamModule private helper。

```text
A. policy A 编译并发布 P；provider 返回 P。
B. 不改变 source geometry，应用 policy B。
C. provider 不再把 P 标为 current committed。
D. 带 A key 的迟到结果被丢弃。
E. B key 编译成功，一次发布 P2；provider 返回 P2。
F. 重复读取 P2：plan/context/report 配套；optimizer 调用计数不增加。
G. 同 hash proof-only 更新：motion 内容和选择不变。
```

mode、DOF flag、Z flag 至少覆盖当前真实可变路径；capability/service 尚不可用时使用明确隔离的 test-only authority 验证代际，不伪造生产资格。

### 5.4 完成条件

旧 context 自洽不再等于 current；公开 provider、直接导出和准备阶段使用同一 readiness 事实。无 partial publish、无旧 policy 冒充新 policy、无 raw catalog 冒充 committed plan。

## 6. 同批低成本加固与可延后优化

### 6.1 建议同步加固 finalizer

校验 physical layout、active mask、MotionClass 和优化状态的组合。不允许通过把 `Reduced` 标签改成 `Optimized` 绕过 inactive-axis hold；同时不得把真正三轴/四轴机误判为非法五轴降维。

需要拒绝的是语义自相矛盾的输入，而不是所有低轴数 Block。补一个 mask 不覆盖实际变化轴、一个 class/mask 不一致、一个 source span 索引越界的反例即可；不要求重跑整个 B0。

依据：[C12 · FinalMotionPlan finalizer / 身份检查](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/project/cam/final_motion_plan.cpp#L205-L344)；[C9 · ReductionPolicy / Admission / ProcessZEnvelope](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/src/core/algorithms/cam/dof_reduction.h)。

### 6.2 性能优化顺序

优先级：编译一次/发布一次 → per-plan/per-block 绑定复用 → worker 私有 FK 状态复用 → candidate invariant cost 复用 → 直接旋转分段数/最大 affine merge。

保留已完成的 `evaluatePhysicalKnots()`；不要为重构撤回这次批量求值收益。先计数调用和测量，再声明加速，不设置没有依据的新性能门槛。

这些不各自占用验收节点；没有影响 correctness 的优化可以后移，写入 Deferred 即可。

## 7. C3 — 一次完成 B1 Stage Gate

### 7.1 运行原则

这是原规划已有的正式 Gate，不是新增验收。将 C1/C2 新测试和已有 S1/S2/S3 suite 合并去重执行一次。开发过程中允许按需要跑 targeted tests，禁止要求每个 helper 都交一份完整报告。

复用用户现有可运行环境和已有 build presets；不重新安装 SDK、不要求重新建立工具链。先用既有构建目录的 CTest inventory 确认 target 名称，再运行同一批次测试集合。

### 7.2 V-row 映射与最低证据

| V-row | 最低证据 | 本轮注意事项 |
|---|---|---|
| V-007 Geometry normalization | duplicate、winding、真实 corner/fence 不丢 | 带入 C1 的结构分块≠工艺事件测试。 |
| V-008 OCC-first | line/arc/circle 来源保留；false fit 不晋升 | 不要求 native GTN arc 指令。 |
| V-009 Rotary continuity | 359→361、351→711、限位与周期 source | 保留 S2 closure/entry 回归。 |
| V-010 Full5D | strict denoise / 派生重建；无 tolerance 不改工艺意义 | 未支持 relaxed smoothing 标为已批准的 Deferred，不伪称全能力通过。 |
| V-011 Resampling | hard rotary-step、硬预算、真实 bound 缺失处理 | dynamics audit-only 按已批准范围标注，不以统计冒充控制器规划。 |
| V-012 Merge | strict affine 正例；midpoint/fence/unknown 反例 | C1 事件投影在 merge 后保持。 |
| V-013 DOF | C-only/U+C/3D/4D、false C-only、1001 tiny increments | 至少一条真实 frozen FK 短轨迹搭配 test-only admission；生产 unqualified 单列。 |
| V-014 Z | 数值 Z、末边界不变、真实解析 bound；无 process envelope 拒绝 | 不要求生产工艺 Z 白名单。 |
| V-015 Selection | 相同输入输出 hash/class 一致；资格/模式/动态不匹配拒绝 | 不能用测试资格冒充设备资格；成本 stops 使用 C1 正式语义。 |
| V-016 Publication | plan/context/report 原子；真实 provider 失效；late/cancel 拒绝 | 必须走 C2 公共服务入口，不只测 helper。 |

原矩阵：[P5 · 批次验证矩阵 V-007～V-016](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/validation/validation-matrix.md)。

### 7.3 复用的已知测试 targets

以下名称来自当前仓库实施记录，执行者应在本地 CTest inventory 中再次匹配，不自行发明 target：

```text
lcnc_dof_reduction_test
lcnc_full5d_optimizer_test
lcnc_cam_algorithm_pipeline_test
lcnc_cam_motion_foundation_test
lcnc_cam_motion_plan_contract_test
lcnc_cam_lead_in_test
lcnc_project_package_test
```

可以将新增测试并入对应 target。配置/缓存/事件合同直接影响的现有回归一并加入；不为了形式执行 B2/B3/B4 测试矩阵。

命令模板（`$BuildDir` 指用户已经可运行的实际构建目录）：

```powershell
ctest --test-dir $BuildDir -N
ctest --test-dir $BuildDir --output-on-failure -R '^lcnc_(dof_reduction|full5d_optimizer|cam_algorithm_pipeline|cam_motion_foundation|cam_motion_plan_contract|cam_lead_in|project_package)_test$'
```

上面只给出去重后的已知 suite，不等于默认覆盖了全部 V-row；必须由 test-case→V-row 表确认实际断言。构建命令复用仓库实际 preset，不以规划猜测配置名称。

### 7.4 Metrics 与资格分层

每条代表路径记录：输入/输出 knots、blocks、实际参与轴、最大位姿偏差、真实工艺事件数、旋转 travel/reversal、候选拒绝原因、compile time、planHash。

至少区分：

```text
生产 unqualified / Off：输出是参考计划，允许没有降维收益。
生产 unqualified / Conservative或Full：可做已支持 strict Full5D；降维 admission 拒绝。
测试 qualified fixture：验证降维选择与数值 Z 算法；不代表实机可用。
```

没有实际测量就填 Not measured，不填 0；无 bound 的功能填 Unsupported/Deferred，不填 zero-error certified。

### 7.5 失败与重跑

失败先归类为功能、数据、构建环境或性能波动；禁止放宽阈值只为绿灯。功能修复后只重跑受影响 test 及直接依赖；汇总中记录初次失败与重跑 commit。最终 Gate 证据必须与最终收口代码可对应，不把历史7/7拼成没有运行过的一次全绿。

ASan 可按本次改动范围运行现有定向 suite；不要求重复整套 Release/ASan/HIL 全矩阵。整个正式 Gate 只汇总一次。

## 8. Boolean Exit

```text
[ ] F01：纯 source/block 划分不生成真实 LaserOff/On/Stop。
[ ] F01：真实工艺事件与 entry ownership 保留，成本 stop 语义正确。
[ ] F02：公开 committed provider 按当前 authority key/epoch 判定有效性。
[ ] F02：配置重读/应用、迟到结果与取消不会发布或返回旧计划为 current。
[ ] F02：plan/context/report 作为完整 record 发布，重复读取不重新选择。
[ ] F03：V-007～V-016 一次 Gate 有 test-case 映射与实测证据。
[ ] 生产 Unavailable/revision 0 仍未被升级成 qualification。
[ ] 所有 test-only qualification 与生产来源隔离。
[ ] 不依赖 collision 后端，不新增第二 SDK queue。
[ ] source seam、多圈、fence、数值预算不回退。
[ ] Deferred 范围与主规划/功能规划/验证矩阵一致。
[ ] 无未解决的 B1 correctness blocker。
```

通过后执行模型只能记录 `B1 Stage Gate Passed / R1 Review Pending`。不能自行伪造 Astra R1 PASS。

## 9. 完成证据：只交一份汇总

```text
审计基线与最终 commit
C1/F01 修改面与真实事件反例
C2/F02 public provider 失效/late/cancel 结果
可选 finalizer / 性能改进
test→V-row 映射、命令、结果、失败与重跑
before/after metrics 与资格来源标签
Deferred / 不支持范围
请求 R1 复核
```

主规划、执行记录和 STATUS-MATRIX 同步到真实已提交状态；不再写“未推送工作区”。不需要逐任务 evidence 文件，也不需要重述整包架构。

## 10. R1 复核提示词

```text
审阅 codex/cam-motion-v3.2.1-b1 最终收口 HEAD。
对照 B1_R1_AUDIT_REPORT.md 和本执行计划。

重点：
1. source/block 结构边界是否已与激光状态/required stop 分离；
2. Raw→Full5D→Reduction 真实工艺事件是否等价；
3. ICamToolpathProvider 公共读取是否拒绝旧 policy/context 缓存；
4. owner-key、worker-key、已发布 record 是否一致，late/cancel 是否事务性；
5. V-007～V-016 Stage Gate 证据是否与本次最终代码匹配；
6. S1/S2 已批准 strict subset 没有被伪造为更大能力；
7. 生产缺 qualification 与 process envelope 是否真实保留；
8. 是否存在新的阻断 B2 消费合同的问题。

不要求为本轮增加 HIL、实机、通用平滑、通用区间证明或生产碰撞验收。
输出 PASS / PASS_WITH_PATCH / BLOCKED，列明实际 blocker。
只有问题关闭且 Gate 完成时写：R1=PASS；B2_RELEASE=YES。
```

## 11. B2 交接合同

B2 只消费最终固定 plan/block/context 与工艺事件，不重新 IK、简化、降维或换候选。未资格快照可以用于离线开发和明确隔离的 trace 测试；任何真实设备支持单元仍须经过后续 controller qualification。

本次放行只代表 B1 软件交付满足 B2 开发前置，不表示 B2/B3/B4 已完成，更不表示激光实机生产许可。

依据：[P4 · B2 Process / GTN 执行边界](https://github.com/super1wang/LaserCNCv1.0/blob/aa8fd34625bca3581898915b12140f9cc5510cf1/docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/B2-process-gtn-execution.md)。
