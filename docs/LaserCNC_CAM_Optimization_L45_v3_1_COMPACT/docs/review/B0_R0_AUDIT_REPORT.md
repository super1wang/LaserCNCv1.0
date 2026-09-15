# LaserCNC CAM Motion v3.1 — B0 / Astra R0 审计报告

> 审计对象：`super1wang/LaserCNCv1.0`
> 审计分支：`codex/cam-motion-v3.1-b0`
> 审计 HEAD：`fcc3d173dc82faa8ae7d60d7ea46593cc1615081`
> B0 主实现提交：`6b11aebd68ddaf1fddc56d46cd4afd3945c6899e`
> B0 开发基线：`815ceb8ecd03ebbba5119c04693e64b8d644439a`
> 规划基线：`4254db59696193963392e2946132cf614c8def26`
> 审计结论：**PASS_WITH_PATCH**
> B1 状态：**暂不放行；关闭本报告列出的收口项后放行**

---

## 1. 审计目的

本次审计只回答一个问题：

> B0 是否已经建立了足够稳定的 Collision Policy / FinalMotionPlan / ContinuousMotionEvaluator 基础，使 B1 可以在不重新定义基础语义的情况下进入 Geometry → Full5D → Resampling → Merge → DOF Reduction → Publication 开发？

审计重点不是继续扩大 B0，也不是要求生产/HIL 验收，而是确认 B1 将依赖的基础运动语义是否闭合。

---

## 2. 审计依据

### 2.1 规划依据

B0 计划要求冻结：

- `MotionClass`
- `ControllerMotionMode`
- `CollisionVerificationMode`
- 单一 `FinalMotionPlan`
- collision-independent `ContinuousMotionEvaluator`
- Disabled 零碰撞后端依赖
- Required fail-closed
- 三个策略维度正交
- evaluator/model version 进入计划身份

B1 明确依赖 B0：

- Full5D 优化必须基于 B0 的 FinalMotionPlan / evaluator；
- adaptive resampling 和 5D merge 必须使用共享 evaluator；
- reduced candidate 必须从 Optimized Full5D 产生；
- B0 contract 在 B1 中视为冻结输入，除非触发 ESCALATE。

### 2.2 仓库事实依据

审阅的主要源码/测试包括：

```text
src/core/project/cam/collision_validation_contracts.h
src/core/project/cam/final_motion_plan.cpp
src/core/algorithms/cam/continuous_motion_evaluator.h
src/core/algorithms/cam/continuous_motion_evaluator.cpp
src/modules/cam/toolpath/cam_module_toolpath.cpp
src/modules/cam/collision/continuous_motion_certificate_builder.cpp
src/modules/cam/settings/cam_config.cpp
src/modules/cam/contracts/toolpath_export_dto.h
src/modules/cam/contracts/i_cam_initial_approach_planner.h
src/modules/process/runtime/process_cutting_safety.cpp

tests/cam_motion_foundation_test.cpp
tests/cam_motion_plan_contract_test.cpp
tests/process_cutting_safety_test.cpp

docs/versions/2026-09-14-cam-motion-v3.1-b0.md
docs/LaserCNC_CAM_Optimization_L45_v3_1_COMPACT/docs/plans/cam-motion-v3.1/*
```

---

## 3. 总体结论

### 3.1 已满足的 B0 主目标

| 目标 | 审计结论 |
|---|---|
| MotionClass / ControllerMotionMode / CollisionVerificationMode 正交 | PASS |
| FinalMotionPlan context/block/plan 哈希与 stale 检测 | PASS |
| legacy `nodes` 被定义为派生/兼容视图 | PASS，仍需 B2 最终消费者切换 |
| Disabled 不再被 Process 主碰撞门禁阻断 | PASS |
| Disabled certificate builder 进入 worker/geometry 前短路 | PASS |
| Required 基本 fail-closed | PASS_WITH_PATCH |
| ContinuousMotionEvaluator 与碰撞后端解耦 | PASS |
| PhysicalAxes evaluator 不独立线性插值 TCP | PASS |
| RTCP evaluator 要求外部提供 qualified semantics | PASS |
| B0 交付的本地构建与定向测试记录 | ACCEPTED AS EVIDENCE |

B0 的主体架构方向正确，不需要回滚或重做。

### 3.2 放行判断

```text
R0_RESULT = PASS_WITH_PATCH
B1_RELEASE = BLOCKED_UNTIL_PATCH_CLOSED
```

原因不是“测试不够多”，而是存在少数会直接污染 B1 数值优化正确性的基础语义缺口。

---

# 4. 阻断项

## B0-F01 — P1：跨 Block 运动边未被 ContinuousMotionEvaluator 完整表示

### Repository Fact

`attachMotionPlan()` 以 `phase / contourId / rapidPhase` 变化创建新 `CamMotionBlock`。新 Block 从当前 node 开始，不显式携带上一 Block 的末端运动边界。

`ContinuousMotionEvaluator::evaluate()` 仅在单个 `CamMotionBlock::physicalKnots` 内插值。

因此以下合法序列：

```text
LeadIn(P0) -> Cutting(P1) -> Cutting(P2)
```

可能形成：

```text
Block A: LeadIn [P0]
Block B: Cutting [P1, P2]
```

于是：

```text
Block A evaluator: 只覆盖 P0
Block B evaluator: 覆盖 P1 -> P2
```

`P0 -> P1` 没有归属于任何一个可连续求值的 Block。

### 风险

B1 的：

- adaptive resampling
- 5D merge
- continuous equivalence
- DOF reduction
- 后续 exact-plan simulation / collision proof

都将依赖 block evaluator。

如果运动边 ownership 未冻结，B1 会在阶段边界产生“旧 nodes 有边、Block evaluator 无边”的双语义。

### Required Fix

必须冻结一种且仅一种边 ownership 规则，并保证：

```text
每一条 canonical motion edge
  被一个且仅一个 semantic block/evaluator interval 覆盖
```

允许的实现形式包括：

1. Block 显式携带 entry boundary / predecessor state；或
2. 相邻 Block 共享边界 knot，但 projection/fence/hash 语义必须定义清楚；或
3. 引入 canonical edge/span 作为 Block 的正式区间单位。

不得：

- 靠消费者“猜上一节点”；
- optimizer 使用一种边界语义、Process/Simulation 使用另一种；
- 用 legacy `nodes` 补洞并继续称 Block 为唯一执行真相。

### Required Test

至少构造：

```text
LeadIn -> Cutting
Cutting -> Rapid(Retract)
Retract -> Traverse
Traverse -> Approach
Approach -> Cutting
```

断言：

- 原始 canonical edge count == evaluator coverage edge count；
- 无缺边；
- 无重复边；
- phase/process fence 边界保持；
- plan hash 对边界语义变化敏感。

---

## B0-F02 — P1：Evaluator bound 合法性与模型身份绑定不足

### Repository Fact

当前 `bound()` 主要检查：

```text
callback success
result.valid == true
maximumPositionErrorMm finite
maximumOrientationErrorDegrees finite
```

但没有完整检查：

- TCP min/max 是否 finite；
- physical axis bounds 是否 finite；
- min <= max；
- error 是否非负；
- callback context 的 interpolation model version 是否与计划声明一致。

### 风险

B1 会把 `bound()` 当成 adaptive resampling / merge 的证明入口。

若非法区间可以通过，后续优化可能将：

```text
NaN / inverted interval / negative error
```

当作“已证明的保守区间”。

同时，`interpolationModelVersion` 虽已进入 plan hash，但 evaluator API 只验证“非零”，不能证明正在使用的 evaluator context 与该计划身份一致。

### Required Fix

建立一次性绑定后的 evaluation context：

```text
BoundMotionEvaluationContext
  plan/context identity
  interpolation model version
  kinematic/controller semantic identity
  callbacks
```

建议：

- 在 plan/compiler 或 evaluator factory 边界完成一次绑定；
- 单次 `evaluate/bound` 不重复计算 plan hash；
- `bound()` 返回前做最小但完整的结构合法性检查。

### Required Validation

必须拒绝：

```text
model version mismatch
NaN/Inf bounds
min > max
negative position/orientation error
invalid source parameter
unsupported interpolation
```

必须保留当前 midpoint 非线性 TCP 反例。

---

## B0-F03 — P1：Optional 在 initial approach 内部仍可能成为路径否决/候选选择条件

### Repository Fact

Process 主门禁和 `InitialApproachSnapshot::isExecutable()` 已对 Optional 放行。

但 `planInitialApproach()` 内部接受 candidate 时仍根据：

```text
collisionResult.complete
&& !collisionResult.blocksExecution(...)
```

决定是否返回该 transition，并可能根据碰撞结果继续搜索其他 safety-Z candidate。

### 风险

`Optional` 的定义是诊断模式：

```text
可附加诊断
不得成为 execution gate
不得改变已经由非碰撞语义确定的运动候选
```

当前实现可能出现：

```text
候选通过 kinematics/limits
  -> Optional collision diagnostic 返回 Collision/Unknown
  -> 原候选被放弃
  -> 选择另一 safety-Z 或整体失败
```

这让 collision policy 实际重新参与路径选择。

### Required Fix

在 initial approach 中严格分离：

```text
path admissibility
vs
collision diagnostic/certification
```

策略必须是：

```text
Disabled:
  不运行 collision certifier
  不因 collision state 改候选

Optional:
  路径选择不依赖 collision result
  可附加 diagnostics
  diagnostics 不改 candidate

Required:
  collision proof 是最终 eligibility gate
```

### Required Test

构造同一条运动学合法的 initial approach：

```text
Disabled + 无几何资源 -> 同一路径通过
Optional + 诊断 Collision/Unknown -> 路径候选不改变
Required + Collision/Unknown -> fail-closed
```

---

## B0-F04 — P1：Required 路径对 `Disabled` certificate 的资格判定不够严格

### Repository Fact

`CamMotionEdgeCertificate::executionEligible()` 当前对：

```text
CertifiedSafe
Disabled
```

均返回 true。

而 Required 分支仍复用该通用 helper。

### 风险

在状态组合异常或旧数据迁移下，Required 可能把“未做碰撞认证”的 Disabled certificate 当作“执行可接受”。

### Required Fix

不要让通用 helper 替代策略判断。

正式规则冻结为：

```text
Disabled / Optional:
  certificate 不作为 execution eligibility 前置

Required:
  required edge certificate MUST == CertifiedSafe
```

可保留通用 `executionEligible()` 用于显示/兼容，但 Required gate 必须显式要求 `CertifiedSafe`，或者将 API 改为 policy-aware eligibility。

### Required Test

```text
Required + Disabled certificate        -> BLOCK
Required + BoundaryUnknown             -> BLOCK
Required + Blocked                     -> BLOCK
Required + CertifiedSafe all edges     -> PASS
Disabled + no certificates             -> 不因碰撞证书阻断
Optional + no/failed diagnostic proof  -> 不因碰撞证书阻断
```

---

## B0-F05 — P2：非法新 collision mode 字符串会静默回退为 Disabled

### Repository Fact

`collisionVerificationModeFromString(value, legacyEnabled)`：

- 合法字符串按新策略解析；
- 其他值回退到 legacy bool；
- 新格式写回时已经删除旧 `collisionDetectionEnabled`。

因此：

```toml
collisionVerificationMode = "requried"
```

可能被解析成 Disabled，而不是配置错误。

### 风险

这是一个低成本但安全方向错误的 silent downgrade。

### Required Fix

解析必须区分：

```text
A. 新字段缺失
B. 新字段存在且合法
C. 新字段存在但非法
```

推荐语义：

```text
A: 按 legacy migration
B: 使用显式新值
C: 配置 invalid / fail-closed，不得自动 Disabled
```

### Required Test

```text
legacy enabled=true            -> Required
legacy enabled=false           -> Disabled
new "disabled"                 -> Disabled
new "optional"                 -> Optional
new "required"                 -> Required
new invalid string             -> invalid/fail-closed
```

---

# 5. 非阻断但高收益优化

## O-01 — B1 必须替换过渡性质的 MotionCompilationContext 值

当前 B0 的 context 中仍存在过渡值，例如：

```text
workspaceGeneration ~= snapshot.revision
setupCalibrationHash ~= machineKinematicsHash
controllerMode = PhysicalAxes
controllerCapabilityHash ~= axis name/role digest
toolProcessHash = limited tool/offset fields
```

这些值足够支持 B0 contract 验证，但不能被当作 B1/B2 最终生产身份。

**处置：不阻断 B0 收口；在 B1 Workstream 7 的 immutable compile context capture 中替换真实来源。**

---

## O-02 — finalizeMotionPlan 集中补结构 invariant，减少 B1 重复验证

建议在 finalizer/plan validator 中一次检查：

```text
blockId uniqueness
source span index range / monotonicity
fence index validity
non-negative duration / tolerance error
activeAxisMask consistency
block interpolation required data availability
```

这样 B1 optimizer 内部无需重复做全局结构检查。

---

## O-03 — 新增少量组合反例，不扩大测试矩阵

当前测试对 helper 级别覆盖不错。

后续只需要补：

```text
block boundary coverage
Optional initial approach non-gating
Required rejects Disabled certificate
invalid config does not downgrade
evaluator model mismatch / invalid bound
```

不建议为 B0 再跑完整 Release/ASan/HIL/生产矩阵。

---

# 6. B0 收口 Gate

只有以下条件全部满足，R0 才从 `PASS_WITH_PATCH` 转为 `PASS`：

```text
[ ] F01 canonical motion edge coverage 完整且唯一
[ ] F02 evaluator model identity 绑定，非法 bound 被拒绝
[ ] F03 Optional 不参与 initial approach path selection/gating
[ ] F04 Required 只接受 CertifiedSafe certificate
[ ] F05 invalid collisionVerificationMode 不可 silent downgrade
[ ] 直接影响的现有 contract/safety/no-model tests 通过
[ ] 无新增第二 motion truth
[ ] 无新增 DeviceCommandQueue / SDK bypass
[ ] B0 状态文档更新为 Astra PASS
[ ] 无 unresolved ESCALATE
```

---

# 7. 推荐放行结论

```text
B0 implementation direction: ACCEPT
B0 architecture: ACCEPT
B0 regression evidence: ACCEPT FOR B0 SCOPE

R0: PASS_WITH_PATCH
B1: HOLD

Expected closure:
one compact patch batch
one targeted regression
one Astra R0 re-review

No full B0 rerun required.
```

修复后，不建议继续在 B0 扩展功能，应立即进入 B1。
